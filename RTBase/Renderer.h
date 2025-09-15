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

struct VPL
{
	ShadingData shadingData;
	Colour      Le;
	bool        onLight;
	const Triangle* tri = nullptr;
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
	int lastNumLightPaths = 0;

	std::vector<Colour> accumulator;
	std::vector<Colour> accumulatorSqr;
	std::vector<int>    sampleCount;

	std::mutex accumMutex;

	std::atomic<int> nextTile;
	std::atomic<int> activeWorkers;

	oidn::DeviceRef device;
	oidn::BufferRef colourBuf;
	oidn::FilterRef filter;

	std::vector<VPL> vpls;

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

	bool firstNonSpecularHit(Ray r, Sampler* sampler, ShadingData& outSD) 
	{
		for (int depth = 0; depth <= MAX_DEPTH; ++depth) 
		{
			IntersectionData isect = scene->traverse(r);
			ShadingData sd = scene->calculateShadingData(isect, r);

			if (sd.t >= FLT_MAX) return false;        

			if (!sd.bsdf->isPureSpecular()) 
			{
				outSD = sd;
				return true;                         
			}

			//otherwise bounce specularly
			Colour f; float pdf;
			Vec3 wi = sd.bsdf->sample(sd, sampler, f, pdf);
			if (pdf < EPSILON) 
				return false;

			r.init(sd.x + wi * EPSILON, wi);
		}
		return false;
	}

	//to clamp very bright values to prevent bright spots
	Colour clampMax(const Colour& c, float maxValue = 20.0f)
	{
		return Colour(min(c.r, maxValue),min(c.g, maxValue),min(c.b, maxValue));
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
			GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(Dot(-wi, light->normal(shadingData, wi)), 0.0f)) / l;

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

	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float px, py;
		if (!scene->camera.projectOntoCamera(p, px, py))
			return;

		if (px < 0.0f || py < 0.0f || px >= (float)scene->camera.width || py >= (float)scene->camera.height)
			return;

		Vec3 wi = (scene->camera.origin - p).normalize();        
		Vec3 nCam = (-scene->camera.viewDirection).normalize();  
		float cosTheta = max(0.0f, Dot(wi, nCam));

		float We = 1.0f / (scene->camera.Afilm * powf(max(1e-6f, cosTheta), 4.0f));

		int ix = (int)px;
		int iy = (int)py;
		int index = iy * scene->camera.width + ix;

		const Colour wcol = clampMax(col * We);
		std::lock_guard<std::mutex> lk(accumMutex);
		accumulator[index] = accumulator[index] + wcol;
		accumulatorSqr[index] = accumulatorSqr[index] + (wcol * wcol);
		sampleCount[index] += 1;
	}

	//1.sample the light source
	//2.sample a position from the light
	//3.sample the direction from  the light
	//4.get the colour of the light
	//5.connect to the camera
	//6.trace the path using the position and the direction
	//7.recursively trace the path
	void lightTrace(Sampler* sampler)
	{
		float pmf = 0.0f;
		float pdfPosition = 1.0f;
		float pdfDirection = 1.0f;

		Light* sourceLight = scene->sampleLight(sampler, pmf);
		if (!sourceLight || pmf <= EPSILON) return;

		Vec3 position = sourceLight->samplePositionFromLight(sampler, pdfPosition);
		Vec3 direction = sourceLight->sampleDirectionFromLight(sampler, pdfDirection);
		if (pdfPosition <= EPSILON || pdfDirection <= EPSILON) return;

		Colour lightColour = sourceLight->evaluate(direction);

		Colour pathThroughput = lightColour / (pmf * pdfPosition * pdfDirection);

		Vec3 wiCam = (scene->camera.origin - position).normalize();
		const Triangle* ignoreTri =
			sourceLight->isArea() ? static_cast<AreaLight*>(sourceLight)->triangle : nullptr;

		if (scene->visible(position + wiCam * EPSILON, scene->camera.origin - wiCam * EPSILON, ignoreTri))
		{
			connectToCamera(position, direction, pathThroughput);
		}

		Ray ray(position + direction * EPSILON, direction);
		lightTracePath(ray, pathThroughput, lightColour, sampler, 0);
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler,int depth)
	{
		if (depth >= MAX_DEPTH)
		{
			return;
		}

		//trace the ray to find the next intersection
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t >= FLT_MAX)
		{
			return; //ray left the scene
		}

		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		//direction from point to camera
		Vec3 wi = scene->camera.origin - shadingData.x;
		wi.normalize();

		const Triangle* ignoreTri = intersection.tri;

		if (scene->visible(shadingData.x + wi * EPSILON, scene->camera.origin - wi * EPSILON, ignoreTri))
		{

			Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			float cosSurf = max(0.0f, Dot(shadingData.sNormal, wi));
			Colour col = pathThroughput * bsdf * cosSurf;
			//Colour col = pathThroughput * bsdf;
			connectToCamera(shadingData.x, shadingData.sNormal, col);
		}

		//russian Roulette to terminate the path
		float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() >= russianRouletteProbability)
		{
			return; 
		}
		pathThroughput = pathThroughput / russianRouletteProbability;

		//sample a new direction from the BSDF to continue the path
		Colour bsdfSample;
		float pdf;
		Vec3 wiNew= shadingData.bsdf->sample(shadingData, sampler, bsdfSample, pdf);

		//if sampling failed,terminate the path
		if (pdf < EPSILON || bsdfSample.Lum() < EPSILON)
		{
			return;
		}

		pathThroughput = pathThroughput * bsdfSample * fabsf(Dot(wiNew, shadingData.sNormal)) / pdf;

		r.init(shadingData.x + (wiNew * EPSILON), wiNew);

		lightTracePath(r, pathThroughput, Le, sampler, depth + 1);
	}

	void traceVPLs(Sampler* sampler, int N_VPLS) 
	{
		vpls.clear();
		vpls.reserve(N_VPLS * 4); 

		for (int i = 0; i < N_VPLS; ++i) 
		{
			float pmf = 0.0f, pdfPos = 1.0f, pdfDir = 1.0f;
			Light* L = scene->sampleLight(sampler, pmf);
			if (!L || pmf <= EPSILON) continue;

			Vec3  pL = L->samplePositionFromLight(sampler, pdfPos);
			Vec3  wi0 = L->sampleDirectionFromLight(sampler, pdfDir);
			if (pdfPos <= EPSILON || pdfDir <= EPSILON) continue;

			//source emission in sampled direction
			Colour Le0 = L->evaluate(wi0);             

			//initial throughput
			Colour pathThroughput = Le0 / (pmf * pdfPos * pdfDir * float(N_VPLS));

			if (L->isArea()) 
			{
				auto* AL = static_cast<AreaLight*>(L);
				VPL v;
				v.shadingData = ShadingData(pL, AL->triangle->n);  //true surface normal
				v.shadingData.bsdf = nullptr;
				v.Le = pathThroughput;
				v.onLight = true;
				v.tri = AL->triangle;                         
				vpls.push_back(v);
			}

			Ray r(pL + wi0 * EPSILON, wi0);
			VPLTracePath(r, pathThroughput, Le0, sampler, 0);
		}
	}

	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) 
	{
		if (depth >= MAX_DEPTH) return;

		IntersectionData isect = scene->traverse(r);

		if (isect.t >= FLT_MAX) 
			return; //left the scene

		ShadingData sd = scene->calculateShadingData(isect, r);   

		//store a VPL at this surface (skip purely specular)
		if (!sd.bsdf->isPureSpecular()) 
		{
			VPL v;
			v.shadingData = sd;              
			v.Le = pathThroughput;  //Le_vpl = throughput * Le
			v.onLight = false;
			v.tri = isect.tri;
			vpls.push_back(v);
		}

		//russian roulette
		float rr = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() >= rr) return;
		pathThroughput = pathThroughput / rr;

		//sample next direction at the current surface
		Colour f;
		float  pdf;
		Vec3   wi = sd.bsdf->sample(sd, sampler, f, pdf);
		if (pdf < EPSILON || f.Lum() < EPSILON) return;

		pathThroughput = pathThroughput * f * fabsf(Dot(wi, sd.sNormal)) / pdf;
		r.init(sd.x + wi * EPSILON, wi);
		VPLTracePath(r, pathThroughput, Le, sampler, depth + 1);
	}

	inline float geometryTerm(const Vec3& x, const Vec3& nx,const Vec3& y, const Vec3& ny, bool useNyCos, Vec3& dir_xy) const
	{
		dir_xy = (y - x);
		float l2 = dir_xy.lengthSq();
		if (l2 < EPSILON) return 0.0f;
		dir_xy = dir_xy / sqrtf(l2);

		float c1 = max(0.0f, Dot(nx, dir_xy));
		float g = c1 / l2;
		if (useNyCos) g *= max(0.0f, Dot(ny, -dir_xy));
		return g;
	}

	Colour indirectFromVPLs(const ShadingData& hit) 
	{
		Colour L(0.0f, 0.0f, 0.0f);

		for (const VPL& v : vpls) 
		{
			Vec3 dir;
			float G = geometryTerm(hit.x, hit.sNormal, v.shadingData.x,
				v.onLight ? v.shadingData.sNormal : v.shadingData.sNormal, !v.onLight, dir);
			if (G <= 0.0f) continue;

			const Triangle* ignore = v.onLight ? v.tri : nullptr;

			if (!scene->visible(hit.x + dir * EPSILON, v.shadingData.x - dir * EPSILON, ignore))
				continue; //shadowed

			Colour f_x = hit.bsdf->evaluate(hit, dir);

			if (v.onLight) 
			{
				L = L + clampMax(f_x * v.Le * G);
			}
			else 
			{
				Colour f_v = v.shadingData.bsdf->evaluate(v.shadingData, -dir);
				L = L + clampMax(f_x * f_v * v.Le * G);
			}
		}
		return L;
	}

	void renderInstantRadiosity()
	{
		film->incrementSPP();
		const int W = scene->camera.width, H = scene->camera.height;

		const int N_VPLS = 4096;                 
		MTRandom samplerForVPLs(1337);
		traceVPLs(&samplerForVPLs, N_VPLS);

		int tileSize = 32, tilesX = (W + tileSize - 1) / tileSize, tilesY = (H + tileSize - 1) / tileSize;
		std::queue<int> tileQueue; std::mutex queueMutex; std::condition_variable queueCV;
		std::vector<Tile> tiles; tiles.reserve(tilesX * tilesY);
		for (int ty = 0; ty < tilesY; ++ty) for (int tx = 0; tx < tilesX; ++tx) tiles.push_back({ tx * tileSize, ty * tileSize, tileSize, tileSize });
		for (int i = 0; i < (int)tiles.size(); ++i) tileQueue.push(i);
		std::atomic<bool> stop(false);

		auto worker = [&](int threadIndex)
			{
				MTRandom* s = &samplers[threadIndex];
				while (true) 
				{
					int idx = -1;
					{
						std::unique_lock<std::mutex> lk(queueMutex);
						if (tileQueue.empty()) break;
						idx = tileQueue.front(); tileQueue.pop();
					}

					Tile t = tiles[idx];
					for (int y = t.startY; y < min(H, t.startY + t.height); ++y) 
					{
						for (int x = t.startX; x < min(W, t.startX + t.width); ++x) 
						{
							float px = x + s->next(), py = y + s->next();
							Ray   r = scene->camera.generateRay(px, py);                 
							ShadingData sd;
							Colour C(0, 0, 0);

							if (firstNonSpecularHit(r, s, sd)) 
							{
								//direct lighting from real lights (your existing routine)
								Colour Ld = computeDirect(sd, s);                           
								//indirect from all VPLs
								Colour Li = indirectFromVPLs(sd);
								C = Ld + Li;
							}
							else 
							{
								C = scene->background->evaluate(r.dir);
							}

							int index = y * W + x;
							accumulator[index] = accumulator[index] + C;
							accumulatorSqr[index] = accumulatorSqr[index] + (C * C);
							sampleCount[index] += 1;
						}
					}
				}
			};

		int nThreads = std::thread::hardware_concurrency();
		std::vector<std::thread> threads;
		for (int i = 0; i < nThreads; i++) threads.emplace_back(worker, i);
		for (auto& t : threads) t.join();

		//write out to film and canvas (identical to the end of your renderers)
		for (int y = 0; y < H; ++y) 
		{
			for (int x = 0; x < W; ++x) 
			{
				int idx = y * W + x, n = sampleCount[idx];
				if (n > 0) 
				{
					Colour avg = accumulator[idx] / float(n);
					film->splat(x + 0.5f, y + 0.5f, avg);
					unsigned char r, g, b; film->tonemap(x, y, r, g, b, 1, n);
					canvas->draw(x, y, r, g, b);
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

		int basePaths = 10000000;
		float resolutionFactor = sqrt((float)(scene->camera.width * scene->camera.height)) / 50.0f;
		int numLightPaths = (int)(basePaths * resolutionFactor);
		numLightPaths = min(numLightPaths, 10000000);
		lastNumLightPaths = numLightPaths;

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
				const int n = sampleCount[index];

				if (lastNumLightPaths > 0)
				{
					Colour avg = (accumulator[index] / float(n)) * exposureFactor;

					film->splat(x + 0.5f, y + 0.5f, avg);

					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b, 1.0f, n);
					canvas->draw(x, y, r, g, b);
				}
				else
				{

					canvas->draw(x, y, 0, 0, 0);
				}


			}
		}
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
								Colour sampleColour = pathTrace(ray, throughput, 0, sampler);
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