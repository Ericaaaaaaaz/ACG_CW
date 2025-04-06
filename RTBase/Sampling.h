#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here

		float theta = std::acos(r1);         
		float phi = 2.0f * M_PI * r2;        

		float sinTheta = std::sin(theta);
		float x = sinTheta * std::cos(phi);
		float y = sinTheta * std::sin(phi);
		float z = std::cos(theta);           

		return Vec3(x, y, z);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		// check if the sampled direction is in the upper hemisphere
		// in z-up, the hemisphere is the set of directions with z > 0
		if (wi.z > 0.0f)
		{
			
			return 1.0f / (2.0f * M_PI);
		}
		else
		{
			return 0.0f;
		}
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float theta = std::acos(std::sqrt(r1));   
		float phi = 2.0f * M_PI * r2;             

		float sinTheta = std::sin(theta);
		float x = sinTheta * std::cos(phi);
		float y = sinTheta * std::sin(phi);
		float z = std::cos(theta);

		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{

		if (wi.z > 0.0f)
		{
			return wi.z / M_PI;
		}
		else
		{
			return 0.0f;
		}
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		float theta = std::acos(1.0f - 2.0f * r1);  
		float phi = 2.0f * M_PI * r2;               

		float sinTheta = std::sin(theta);
		float x = sinTheta * std::cos(phi);
		float y = sinTheta * std::sin(phi);
		float z = std::cos(theta);

		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / (4.0f * M_PI);
	}
};