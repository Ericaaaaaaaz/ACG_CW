#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <OpenImageDenoise/oidn.hpp>

#define MAX_DEPTH 5

struct Tile
{
	int startX;
	int startY;
	int width;
	int height;
};

//check if pixel is converged
bool isConverged(const Colour& sum, const Colour& sumSqr, int sampleCount, float threshold)
{
	if (sampleCount < 2) return false;

	Colour mean = sum / (float)sampleCount;
	Colour meanSqr = sumSqr / (float)sampleCount;
	Colour var = meanSqr - (mean * mean);

	float maxVar = max(var.r, max(var.g, var.b));
	return (maxVar < threshold);
}

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	bool canHitLight = true;

	std::vector<Colour> accumulator;
	std::vector<Colour> accumulatorSqr;
	std::vector<int>    sampleCount;

	std::atomic<int> nextTile;
	std::atomic<int> activeWorkers;

	oidn::DeviceRef device;
	oidn::BufferRef colourBuf;
	oidn::FilterRef filter;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		clear();

		int imageWidth = scene->camera.width;
		int imageHeight = scene->camera.height;
		accumulator.resize(imageWidth * imageHeight, Colour(0.0f, 0.0f, 0.0f));
		accumulatorSqr.resize(imageWidth * imageHeight, Colour(0.0f, 0.0f, 0.0f));
		sampleCount.resize(imageWidth * imageHeight, 0);
	}
	void clear()
	{
		film->clear();

		std::fill(accumulator.begin(), accumulator.end(), Colour(0.0f, 0.0f, 0.0f));
		std::fill(accumulatorSqr.begin(), accumulatorSqr.end(), Colour(0.0f, 0.0f, 0.0f));
		std::fill(sampleCount.begin(), sampleCount.end(), 0);
	}

	//to clamp very bright values to prevent bright spots
	Colour clampMax(const Colour& c, float maxValue = 20.0f)
	{
		return Colour(
			min(c.r, maxValue),
			min(c.g, maxValue),
			min(c.b, maxValue)
		);
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;

		float pdfLight;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);


		if (!light || pmf < EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		if (pdf <= EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		Vec3 wi;
		float GTerm = 0.0f;

		if (light->isArea())
		{
			wi = p - shadingData.x;
			float l = wi.lengthSq();

			if (l < EPSILON)
				return Colour(0.0f, 0.0f, 0.0f);

			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(Dot(-wi, light->normal(shadingData, wi)), 0.0f)) / l;

			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x + wi * EPSILON, p - wi * EPSILON, static_cast<AreaLight*>(light)->triangle))
				{
					// Shade
					return clampMax(shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf));
				}
			}
			pdfLight = pdf * pmf;
		}
		else
		{
			wi = p;
			GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x + wi * EPSILON, shadingData.x + (wi * 10000.0f), nullptr))
				{
					// Shade
					return clampMax(shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf));
				}
			}
			pdfLight = pdf * pmf;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			if (depth > MAX_DEPTH)
			{
				return direct;
			}

			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			Colour bsdf;
			float pdf;
			/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi);

			wi = shadingData.frame.toWorld(wi);

			bsdf = shadingData.bsdf->evaluate(shadingData, wi);*/
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);


			if (pdf < EPSILON || bsdf.Lum() < EPSILON)
			{
				return direct;
			}

			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;


			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler));
		}
		return scene->background->evaluate(r.dir);
	}

	void connectToCamera(Vec3 p, Vec3 n, Colour col, Triangle* ignoreTri = nullptr)
	{
		float px, py;
		if (!scene->camera.projectOntoCamera(p, px, py))
			return;

		//convert to integer pixel coordinates
		int ix = static_cast<int>(px);
		int iy = static_cast<int>(py);

		//check if pixel is within image bounds
		if (ix < 0 || ix >= scene->camera.width || iy < 0 || iy >= scene->camera.height)
			return;

		//distance to camera
		float distSq = (scene->camera.origin - p).lengthSq();
		if (distSq < EPSILON)
			return;

		//check visibility
		Vec3 camDir = (scene->camera.origin - p).normalize();

		float surfCosTheta = max(0.0f, Dot(camDir, n));

		if (surfCosTheta < EPSILON) 
			return;

		if (!scene->visible(p + camDir * EPSILON, scene->camera.origin - camDir * EPSILON, ignoreTri))
			return;

		float camCosTheta = max(0.0f, Dot(-camDir, scene->camera.viewDirection));

		//camera direction should face towards the hit point
		if (camCosTheta < EPSILON)
			return;


		
		float pixelArea = 1.0f / (scene->camera.width * scene->camera.height);
		float sensorFactor = (camCosTheta * surfCosTheta) / distSq;

		int index = iy * scene->camera.width + ix;

		accumulator[index] = accumulator[index] + col * sensorFactor;
		accumulatorSqr[index] = accumulatorSqr[index] + (col * col) * (sensorFactor * sensorFactor);
		sampleCount[index] += 1;
	}

	

	Colour lightTrace(Sampler* sampler)
	{
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		if (!light || pmf < EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		float pdfPosition = 0.0f;
		float pdfDirection = 0.0f;
		Vec3 lightPosition;
		Vec3 lightDirection;

		if (light->isArea())
		{
			lightPosition = light->samplePositionFromLight(sampler, pdfPosition);

	
			Vec3 toCam = (scene->camera.origin - lightPosition).normalize();
			Vec3 lightNormal;

			auto* areaLight = static_cast<AreaLight*>(light);
			lightNormal = areaLight->triangle->gNormal();

			if (Dot(toCam, lightNormal) > 0.1f) 
			{
				float blendFactor = 0.7f;  

				if (sampler->next() < blendFactor) 
				{
					float r1 = sampler->next() * 0.2f; 
					float r2 = sampler->next() * 0.2f;

					Vec3 perturbedCamDir = toCam;
					perturbedCamDir.x += r1 - 0.1f;
					perturbedCamDir.y += r2 - 0.1f;
					lightDirection = perturbedCamDir.normalize();

					pdfDirection = 1.0f; 
				}
				else 
				{
					lightDirection = light->sampleDirectionFromLight(sampler, pdfDirection);
				}
			}
			else 
			{
				lightDirection = light->sampleDirectionFromLight(sampler, pdfDirection);
			}
		}
		else
		{
		
			lightDirection = light->sampleDirectionFromLight(sampler, pdfDirection);

			lightPosition = use<SceneBounds>().sceneCentre + (-lightDirection) * use<SceneBounds>().sceneRadius;

			pdfPosition = 1.0f / (4.0f * static_cast<float>(M_PI) * SQ(use<SceneBounds>().sceneRadius));
		}

		if (pdfPosition < EPSILON || pdfDirection < EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		//start a ray from the light
		Ray ray(lightPosition + lightDirection * EPSILON, lightDirection);

		Colour Le = light->evaluate(lightDirection);

		if (Le.Lum() < EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		Triangle* ignoreTri = nullptr;
		Vec3 nLight;

		if (light->isArea())
		{
			auto* a = static_cast<AreaLight*>(light);
			ignoreTri = a->triangle;
			nLight = a->triangle->gNormal();
		}
		else
		{
			nLight = -lightDirection;
		}

		float cosOnLight = max(0.0f, Dot(nLight, lightDirection));
		if (cosOnLight < EPSILON)
			return Colour(0.0f, 0.0f, 0.0f);

		Colour pathThroughput = (Le * cosOnLight) / (pmf * pdfPosition * pdfDirection);

		

		connectToCamera(lightPosition, nLight, pathThroughput, ignoreTri);

		int rrStartDepth = 2;

		for (int depth = 0; depth < MAX_DEPTH; depth++)
		{
			//intersect the scene
			IntersectionData isect = scene->traverse(ray);
			if (isect.t == FLT_MAX)
			{
				break;
			}

			ShadingData shadingData = scene->calculateShadingData(isect, ray);

			//if hit another light, stop
			if (shadingData.bsdf->isLight())
			{
				connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput);
				break;
			}

			//connect to camera at each valid intersection
			Vec3 camPos = scene->camera.origin;
			Vec3 toCam = (camPos - shadingData.x).normalize();
			float cosTheta = max(0.0f, Dot(toCam, shadingData.sNormal));

			if (cosTheta > EPSILON)
			{
				Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, toCam);

				if (bsdfVal.Lum() > EPSILON)
				{
					connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * bsdfVal);
				}
			}

			if (depth >= rrStartDepth)
			{
				float rrProb = min(max(pathThroughput.Lum(), 0.1f), 0.95f);
				if (sampler->next() > rrProb)
					break;

				pathThroughput = pathThroughput / rrProb;
			}

			//continue the path with proper sampling
			float r1 = sampler->next();
			float r2 = sampler->next();

			//sample new direction in local space
			Vec3 newDirLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);
			float newDirPdf = SamplingDistributions::cosineHemispherePDF(newDirLocal);

			if (newDirPdf < EPSILON)
				break;

			Vec3 newDirection = shadingData.frame.toWorld(newDirLocal);

			//evaluate BSDF for the new direction
			Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, newDirection);

			if (bsdfVal.Lum() < EPSILON)
				break;

			float cosTerm = max(0.0f, Dot(newDirection, shadingData.sNormal));

			pathThroughput = pathThroughput * bsdfVal * cosTerm / newDirPdf;

			//prepare next ray
			ray.init(shadingData.x + newDirection * EPSILON, newDirection);
		}
		return Colour(1.0f, 1.0f, 1.0f);
	}

		Colour direct(Ray& r, Sampler* sampler)
		{
			// Compute direct lighting for an image sampler here
			IntersectionData intersection = scene->traverse(r);
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			if (shadingData.t < FLT_MAX)
			{
				if (shadingData.bsdf->isLight())
				{
					return shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				return computeDirect(shadingData, sampler);
			}
			return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);

	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}



	void renderPathTracing()
	{
		film->incrementSPP();
		int imageWidth = scene->camera.width;
		int imageHeight = scene->camera.height;

	
		//tiling setup
		int tileSize = 32;
		int tilesX = (imageWidth + tileSize - 1) / tileSize;
		int tilesY = (imageHeight + tileSize - 1) / tileSize;

		//create a list of tiles
		std::vector<Tile> allTiles;
		allTiles.reserve(tilesX * tilesY);
		for (int ty = 0; ty < tilesY; ++ty)
		{
			for (int tx = 0; tx < tilesX; ++tx)
			{
				Tile tile{ tx * tileSize, ty * tileSize, tileSize, tileSize };
				allTiles.push_back(tile);
			}
		}


		std::vector<bool> tileDone(allTiles.size(), false);

		float varianceThreshold = 0.001f;
		int   maxPasses = 10;  //maximum number of passes
		int   samplesPerPass = 1; //number of new samples per pixel each pass

		std::queue<int> tileQueue;



		std::mutex queueMutex;
		std::condition_variable queueCV;
		bool stopThreads = false;

		std::atomic<int> tilesRemainingInPass(0);


		std::mutex passDoneMutex;
		std::condition_variable passDoneCV; 

		auto workerFunc = [&](int threadIndex)
			{
				MTRandom* sampler = &samplers[threadIndex];

				while (true)
				{
					int tileIndex = -1;
					{

						std::unique_lock<std::mutex> lock(queueMutex);

						queueCV.wait(lock, [&] {
							return (stopThreads || !tileQueue.empty());
							});

						//if told to stop and no tiles left, exit the thread
						if (stopThreads && tileQueue.empty())
						{
							return;
						}

						//if there is a tile in the queue, pop it
						if (!tileQueue.empty())
						{
							tileIndex = tileQueue.front();
							tileQueue.pop();
						}
						else
						{

							continue;
						}
					}


					Tile& tile = allTiles[tileIndex];
					for (int y = tile.startY; y < tile.startY + tile.height; y++)
					{
						if (y >= imageHeight) break;
						for (int x = tile.startX; x < tile.startX + tile.width; x++)
						{
							if (x >= imageWidth) break;

							Colour tileAccum(0, 0, 0);

							

							for (int s = 0; s < samplesPerPass; s++)
							{
								float px = x + sampler->next();
								float py = y + sampler->next();

								Ray ray = scene->camera.generateRay(px, py);
								Colour throughput(1.0f, 1.0f, 1.0f);
								//Colour sampleColour = pathTrace(ray, throughput, 0, sampler);
								Colour sampleColour = pathTrace(ray, throughput, 0, sampler);
								//Colour sampleColour = direct(ray, sampler);
								tileAccum = tileAccum + sampleColour;

								
							}

							int index = y * imageWidth + x;
							accumulator[index] = accumulator[index] + tileAccum;
							accumulatorSqr[index] = accumulatorSqr[index] + tileAccum * tileAccum;
							sampleCount[index] = sampleCount[index] + samplesPerPass;
						}
					}
					int remaining = tilesRemainingInPass.fetch_sub(1) - 1;
					if (remaining == 0)
					{
						std::unique_lock<std::mutex> lk(passDoneMutex);
						passDoneCV.notify_one();
					}
				}
			};


		int threadCount = std::thread::hardware_concurrency();
		std::vector<std::thread> threads;
		threads.reserve(threadCount);
		for (int i = 0; i < threadCount; ++i)
		{
			threads.emplace_back(workerFunc, i);
		}


		for (int pass = 0; pass < maxPasses; pass++)
		{
			//check if everything is converged at the start
			bool allDone = true;
			for (auto done : tileDone)
			{
				if (!done) { allDone = false; break; }
			}
			if (allDone)
			{
				break;
			}

			int numTilesThisPass = 0;

			//push any tile that isn't converged yet into the queue
			{
				std::unique_lock<std::mutex> lock(queueMutex);
				for (int i = 0; i < (int)allTiles.size(); i++)
				{
					if (!tileDone[i])
					{
						tileQueue.push(i);
						numTilesThisPass++;
					}
				}
			}
			if (numTilesThisPass == 0)
			{

				break;
			}

			tilesRemainingInPass.store(numTilesThisPass);

			queueCV.notify_all();

			{
				std::unique_lock<std::mutex> lk(passDoneMutex);
				passDoneCV.wait(lk, [&] {
					return tilesRemainingInPass.load() == 0;
					});
			}

			if (pass % 2 == 0) //do variance check on even passes
			{
				//variance check
				for (int i = 0; i < (int)allTiles.size(); i++)
				{
					if (tileDone[i])
						continue;

					Tile& tile = allTiles[i];
					bool tileConverged = true;
					for (int y = tile.startY; y < tile.startY + tile.height; y++)
					{
						if (y >= imageHeight) break;
						for (int x = tile.startX; x < tile.startX + tile.width; x++)
						{
							if (x >= imageWidth) break;

							int index = y * imageWidth + x;
							if (!isConverged(accumulator[index],
								accumulatorSqr[index],
								sampleCount[index],
								varianceThreshold))
							{
								tileConverged = false;
								break;
							}
						}
						if (!tileConverged) break;
					}
					tileDone[i] = tileConverged;
				}

			}
		}

		{
			std::unique_lock<std::mutex> lock(queueMutex);
			stopThreads = true;
		}
		queueCV.notify_all();

		for (auto& t : threads)
		{
			t.join();
		}


		int imageWidthFinal = scene->camera.width;
		int imageHeightFinal = scene->camera.height;


		for (int y = 0; y < imageHeightFinal; ++y)
		{
			for (int x = 0; x < imageWidthFinal; ++x)
			{
				int index = y * imageWidthFinal + x;
				int n = sampleCount[index];
				if (n > 0)
				{
					Colour avg = accumulator[index] / (float)n;
					//write to film
					film->splat(x + 0.5f, y + 0.5f, avg);

					
					unsigned char r1, g1, b1;
					film->tonemap(x, y, r1, g1, b1, 1, n);
					canvas->draw(x, y, r1, g1, b1);
				}
			}
		}
	}

	void renderLightTracing()
	{
	
		film->clear();

		std::fill(accumulator.begin(), accumulator.end(), Colour(0.0f, 0.0f, 0.0f));
		std::fill(accumulatorSqr.begin(), accumulatorSqr.end(), Colour(0.0f, 0.0f, 0.0f));
		std::fill(sampleCount.begin(), sampleCount.end(), 0);

		film->incrementSPP();

		//number of light paths to trace - should match the resolution
		//int numLightPaths = scene->camera.width * scene->camera.height * 16; 

		int basePaths = 10000000; 
		float resolutionFactor = sqrt((float)(scene->camera.width * scene->camera.height)) / 50.0f;
		int numLightPaths = (int)(basePaths * resolutionFactor);
		numLightPaths = min(numLightPaths, 10000000);

		float exposureFactor = 3.0f;

		int numThreads = std::thread::hardware_concurrency();
		std::vector<std::thread> threads;
		threads.reserve(numThreads);

		int pathsPerThread = numLightPaths / numThreads;



		auto traceFunc = [&](int threadIndex, int numPaths) 
			{
				MTRandom* sampler = &samplers[threadIndex];
				for (int i = 0; i < numPaths; i++) 
				{
					lightTrace(sampler);
				}
			};

		//launch threads
		for (int i = 0; i < numThreads; i++) 
		{
			int pathCount = (i == numThreads - 1) ?
				numLightPaths - (pathsPerThread * (numThreads - 1)) : pathsPerThread;
			threads.emplace_back(traceFunc, i, pathCount);
		}

		//wait for all threads to complete
		for (auto& t : threads) 
		{
			t.join();
		}


		int imageWidth = scene->camera.width;
		int imageHeight = scene->camera.height;


		for (int y = 0; y < imageHeight; ++y) 
		{
			for (int x = 0; x < imageWidth; ++x) 
			{
				int index = y * imageWidth + x;
				int n = sampleCount[index];

				if (n > 0)
				{
					Colour avg = (accumulator[index] / float(n)) * exposureFactor;

					film->splat(x + 0.5f, y + 0.5f, avg);

					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b, 1.0f, 1);
					canvas->draw(x, y, r, g, b);
				}
				else
				{

					canvas->draw(x, y, 0, 0, 0);
				}

			
			}
		}
	}


	void denoise()
	{
		int width = film->width;
		int height = film->height;

		device = oidn::newDevice();
		device.commit();

		colourBuf = device.newBuffer(width * height * 3 * sizeof(float));
		float* colourData = (float*)colourBuf.getData();

		//fill in the colour buffer with averaged values
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				int index = y * width + x;
				int n = sampleCount[index];
				Colour avg = (n > 0) ? accumulator[index] / (float)n : Colour(0.0f, 0.0f, 0.0f);
				colourData[3 * index + 0] = avg.r;
				colourData[3 * index + 1] = avg.g;
				colourData[3 * index + 2] = avg.b;
			}
		}

		filter = device.newFilter("RT");
		filter.setImage("color", colourBuf, oidn::Format::Float3, width, height);
		filter.setImage("output", colourBuf, oidn::Format::Float3, width, height);
		filter.set("hdr", true);
		filter.commit();
		filter.execute();

		//write denoised result back to film and canvas
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				int index = y * width + x;
				float r = colourData[3 * index + 0];
				float g = colourData[3 * index + 1];
				float b = colourData[3 * index + 2];

				Colour c(r, g, b);
				film->splat(x + 0.5f, y + 0.5f, c);

				/*unsigned char r1 = (unsigned char)(min(1.0f, r1) * 255);
				unsigned char g1 = (unsigned char)(min(1.0f, g1) * 255);
				unsigned char b1 = (unsigned char)(min(1.0f, b1) * 255);
				canvas->draw(x, y, r1, g1, b1);*/
				unsigned char r1, g1, b1;
				int n = sampleCount[index];
				film->tonemap(x, y, r1, g1, b1, 1, n);
				canvas->draw(x, y, r1, g1, b1);
			}
		}
	}



	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};