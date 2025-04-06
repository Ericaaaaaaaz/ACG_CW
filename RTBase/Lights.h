#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3()) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;


	/*Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}*/
	Vec3 sample(const ShadingData& shadingData,Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		Vec3 startPos = shadingData.x;
		Vec3 pos = triangle->sample(sampler, pdf);

		
		float l2 = (startPos - pos).lengthSq();
		Vec3 wi = (startPos - pos).normalize();
		float cosTheta = std::max(Dot(wi, triangle->gNormal()), 0.0f);
		pdf = l2 / (triangle->area * cosTheta);

		return pos;

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) > 0) ///////////////<0
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData,const Vec3& wi, const Vec3& targetPos = Vec3())
	{
		//return 1.0f / triangle->area;
		Vec3 startPos = shadingData.x;
		float l2 = (targetPos - startPos).lengthSq();
		float cosTheta = std::max(Dot(wi, triangle->gNormal()), 0.0f);
		if (cosTheta < EPSILON)
			return 0.0f;
		return l2 / (triangle->area * cosTheta);


	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		//generate two random numbers
		float r1 = sampler->next();
		float r2 = sampler->next();

		//sample direction in local space(around positive z direction)
		Vec3 localDirection = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		pdf = SamplingDistributions::cosineHemispherePDF(localDirection);


		Frame frame;
		frame.fromVector(triangle->gNormal());

		//convert sampled direction to world space
		return frame.toWorld(localDirection);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3())
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	int width;
	int height;
	std::vector<float> pdf;
	std::vector<float>cdf;

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		width = env->width;
		height = env->height;

		pdf.resize(width * height);
		cdf.resize(width * height);

		float sum = 0.0f;

		for (int j = 0; j < height; j++)
		{
			float theta = (float(j) + 0.5f) * M_PI / float(height);
			float sinTheta = sinf(theta);

			for (int i = 0; i < width; i++)
			{
				//luminance of pixel (i,j)
				float L = env->texels[j * width + i].Lum();

				float p = L * sinTheta;
				pdf[j * width + i] = p;
				sum += p;
			}
		}

		float invSum = 1.0f / sum;
		for (int k = 0; k < width * height; k++)
		{
			pdf[k] *= invSum;
		}
		cdf[0] = pdf[0];
		for (int k = 1; k < width * height; k++)
		{
			cdf[k] = cdf[k - 1] + pdf[k];
		}
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		float r = sampler->next();

		int idx = int(std::lower_bound(cdf.begin(), cdf.end(), r) - cdf.begin());
		idx = std::min(idx, width * height - 1);

		int j = idx / width;     // row
		int i = idx % width;     // column

		float u = (i + 0.5f) / float(width);
		float v = (j + 0.5f) / float(height);

		float phi = 2.0f * M_PI * u; 
		float theta = M_PI * v;        

		float sinTheta = sinf(theta);
		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = cosf(theta);
		wi.z = sinTheta * sinf(phi);

		reflectedColour = env->sample(u, v);

		pdf = PDF(shadingData, wi);

		return wi;
		
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;*/
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3())
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.0f)
			phi += 2.0f * M_PI;

		float theta = acosf(wi.y);

		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;

		//compute pixel indices
		int i = int(u * width);
		int j = int(v * height);


		if (i < 0)      
			i = 0;

		if (i >= width) 
			i = width - 1;

		if (j < 0)      
			j = 0;

		if (j >= height)
			j = height - 1;

		
		float p_ij = pdf[j * width + i];

		float sinTheta = sinf(theta);

		if (sinTheta < 1e-4f) 
			sinTheta = 1e-4f; 
		return p_ij / (2.0f * M_PI * M_PI * sinTheta);
		//return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		/*float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;*/
		float total = 0;
		float dTheta = M_PI / height;
		float dPhi = 2.0f * M_PI / width;
		float texelArea = dTheta * dPhi;

		for (int j = 0; j < height; j++)
		{
			float theta = (j + 0.5f) * M_PI / height;
			float sinTheta = sinf(theta);
			for (int i = 0; i < width; i++)
			{
				float L = env->texels[j * width + i].Lum();
				total += L * sinTheta * texelArea;
			}
		}
		return total;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		float r = sampler->next();
		int idx = int(std::lower_bound(cdf.begin(), cdf.end(), r) - cdf.begin());
		idx = std::min(idx, width * height - 1);

		int j = idx / width;
		int i = idx % width;

		//convert to (u,v)
		float u = (i + 0.5f) / (float)width;
		float v = (j + 0.5f) / (float)height;

		float phi = 2.0f * M_PI * u;
		float theta = M_PI * v;

		float sinTheta = sinf(theta);

		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = cosf(theta);
		wi.z = sinTheta * sinf(phi);


		pdf = PDF(ShadingData(), wi);

		return wi;
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;*/
	}

};