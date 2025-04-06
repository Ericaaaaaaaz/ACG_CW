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

#define MAX_DEPTH 5

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	bool canHitLight = true;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
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
		Vec3 p = light->sample(shadingData,sampler, emitted, pdf);
		/*if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
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
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);*/

		if (pdf <= 1e-12f)
			return Colour(0.0f, 0.0f, 0.0f);

		Vec3 wi;
		float GTerm = 0.0f;
		bool visible = false;

		if (light->isArea())
		{
			wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			//visible = (GTerm > 0 && scene->visible(shadingData.x + wi * EPSILON, p));
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
			pdfLight = pdf * pmf;
		}
		else
		{
			wi = p;
			GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			//visible = (GTerm > 0 && scene->visible(shadingData.x + wi * EPSILON, shadingData.x + (wi * 10000.0f)));
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
			pdfLight = pdf * pmf;
		}

		/*if (visible)
		{
			return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
		}*/

		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler,int x =-1,int y=-1)
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

			/*Colour indirect;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);*/

			
			if (isnan(pathThroughput.r) || isnan(pathThroughput.g) || isnan(pathThroughput.b))
			{
				std::cout << "[NaN detected] PathThroughput before recursive pathTrace: ("
					<< pathThroughput.r << ", " << pathThroughput.g << ", " << pathThroughput.b << ")\n";
				return direct;
			}


			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
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
		return scene->background->evaluate(shadingData, r.dir);
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

		Vec3 camDir = (scene->camera.origin - p).normalize();
		float cosTheta = Dot(camDir, scene->camera.viewDirection);
		if (cosTheta <= 0.0f)
			return;

		float geometry = 1.0f / (scene->camera.Afilm * powf(cosTheta, 4));
		film->splat(px, py, col * geometry);
	}

	Colour lightTrace(Sampler* sampler)
	{
		//pmf = probability of choosing this light from all scene lights
		float pmf; 
		//randomly pick a light from the scene light list
		Light* light = scene->sampleLight(sampler, pmf);

		//pdfPosition = probabilty density of choosing the specific position on the light surface
		float pdfPosition = 0.0f;
		//sample a position on the chosen light
		Vec3 lightPosition = light->samplePositionFromLight(sampler, pdfPosition);

		//pdfDirection = probabilty density of choosing the direction from the light surface
		float pdfDirection = 0.0f;
		Vec3 lightDirection = light->sampleDirectionFromLight(sampler, pdfDirection);
		lightDirection = -lightDirection;

		//start a ray from the light
		Ray ray(lightPosition + lightDirection * EPSILON, lightDirection);

		//emitted radiance from the light
		Colour Le = light->evaluate({}, lightDirection);
		Colour pathThroughput = Le * (1.0f / (pmf * pdfPosition * pdfDirection));
		Colour finalColour(0.0f, 0.0f, 0.0f);

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
				
				break;
			}

			
			float px, py;
			bool visibleOnSensor = scene->camera.projectOntoCamera(shadingData.x, px, py);
			if (visibleOnSensor)
			{
				Vec3 toCam = (scene->camera.origin - shadingData.x).normalize();
				float cosTheta = max(0.0f, Dot(toCam, shadingData.sNormal));
				Colour camBSDF = shadingData.bsdf->evaluate(shadingData, toCam);

				connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * camBSDF * cosTheta);

				finalColour = pathThroughput;
				break;
			}
			
			float r1 = sampler->next();
			float r2 = sampler->next();
			Vec3 newDirLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);
			float newDirPdf = SamplingDistributions::cosineHemispherePDF(newDirLocal);

			Vec3 newDirection = shadingData.frame.toWorld(newDirLocal);

			Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, newDirection);

			float cosTerm = max(0.0f, Dot(newDirection, shadingData.sNormal));
			pathThroughput = pathThroughput * bsdfVal * cosTerm / newDirPdf;

			float rrProb = min(0.9f, pathThroughput.Lum());
			if (sampler->next() > rrProb) 
			{
				break;
			}

			pathThroughput = pathThroughput * (1.0f / rrProb);

			ray.init(shadingData.x + newDirection * EPSILON, newDirection);
		}

		//return Colour(0.0f, 0.0f, 0.0f);
		return finalColour;

	}

	void render()
	{
		film->incrementSPP();

		const int numLightPaths = film->width * film->height;

		for (int i = 0; i < numLightPaths; i++)
		{
			// This function starts from the light, randomly
			// bounces, and if it hits the camera, it deposits
			// energy into the correct pixel inside connectToCamera().
			lightTrace(&samplers[0]);
		}

		// Optionally, after the above loop you could do a quick pass
		// over all film pixels to draw them to the canvas so you can
		// see progressive convergence:
		/*for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}*/


		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);

				//white colour
				Colour throughput = Colour(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, throughput, 0, &samplers[0],x,y);


				if (isnan(col.r) || isnan(col.g) || isnan(col.b)) 
				{
					col = Colour(0.0f, 0.0f, 0.0f);
				}

				//Colour col = lightTrace(&samplers[0]);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				

				/*unsigned char r, g, b;

				if (x == 100 && y == 100) 
				{
					Colour pixel = film->film[y * film->width + x];
					std::cout << "Raw film at (100,100): R: " << pixel.r << " G: " << pixel.g << " B: " << pixel.b << std::endl;
				}

				film->tonemap(x, y, r, g, b);
				
				if (x == 100 && y == 100) 
				{
					std::cout << "SPP: " << film->SPP << ", R: " << (int)r << " G: " << (int)g << " B: " << (int)b << std::endl;
				}



				*/
				canvas->draw(x, y, r, g, b);
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