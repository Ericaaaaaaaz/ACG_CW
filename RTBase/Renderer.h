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
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p, static_cast<AreaLight*>(light)->triangle))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f), nullptr))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
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
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler));
		}
		return scene->background->evaluate(r.dir);
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

	

	void render()
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
					Colour avg = accumulator[index]/(float)n;
					//write to film
					film->splat(x + 0.5f, y + 0.5f, avg);

					//write to canvas
					/*unsigned char r = (unsigned char)(min(1.0f, sum.r) * 255);
					unsigned char g = (unsigned char)(min(1.0f, sum.g) * 255);
					unsigned char b = (unsigned char)(min(1.0f, sum.b) * 255);
					canvas->draw(x, y, r, g, b);*/
					unsigned char r1, g1, b1;
					film->tonemap(x, y, r1, g1, b1,1, n);
					canvas->draw(x, y, r1, g1, b1);
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
				Colour avg = (n > 0) ? accumulator[index] / (float)n : Colour(0.0f,0.0f,0.0f);
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
				film->tonemap(x, y, r1, g1, b1, 1,n);
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