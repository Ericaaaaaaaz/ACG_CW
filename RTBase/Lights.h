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
	virtual Colour evaluate(const Vec3& wi) = 0;
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
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		Vec3 startPos = shadingData.x;
		Vec3 pos = triangle->sample(sampler, pdf);

		Vec3 wi = (pos - startPos).normalize();
		float l2 = (pos - startPos).lengthSq();

		float cosTheta = std::max(Dot(-wi, triangle->gNormal()), 0.0f);
		
		if (cosTheta < EPSILON) 
		{
			pdf = 0;
			return pos;
		}
		pdf = l2 / (triangle->area * cosTheta);

		return pos;
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) > 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3())
	{
		Vec3 startPos = shadingData.x;
		float l2 = (targetPos - startPos).lengthSq();
		float cosTheta = std::max(Dot(-wi, triangle->gNormal()), 0.0f);
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
	Colour evaluate(const Vec3& wi)
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

    float normalizationFactor;

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

        if (sum < EPSILON)
        {
            sum = 1.0f;
            //initialize with uniform distribution if sum is too small
            for (int k = 0; k < width * height; k++)
            {
                pdf[k] = 1.0f / (width * height);
            }
        }
        else
        {
            //normalize PDFs
            float invSum = 1.0f / sum;
            for (int k = 0; k < width * height; k++)
            {
                pdf[k] *= invSum;
            }
        }

		//compute CDF
        cdf[0] = pdf[0];
        for (int k = 1; k < width * height; k++)
        {
            cdf[k] = cdf[k - 1] + pdf[k];
        }

        //make sure the last CDF value is exactly 1
        if (cdf[width * height - 1] > 0)
        {
            float invLastCDF = 1.0f / cdf[width * height - 1];
            for (int k = 0; k < width * height; k++)
            {
                cdf[k] *= invLastCDF;
            }
        }

        //store normalization factor for PDF calculation
        normalizationFactor = float(width * height) / (2.0f * M_PI * M_PI);
        //normalizationFactor = 1.0f / (2.0f * M_PI * M_PI);
    }
    Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
    {
        // Assignment: Update this code to importance sampling lighting based on luminance of each pixel
        float r = sampler->next();

        int idx = int(std::lower_bound(cdf.begin(), cdf.end(), r) - cdf.begin());
        idx = std::min(idx, width * height - 1);

        int j = idx / width;     // row
        int i = idx % width;     // column

        float u = (i + sampler->next()) / float(width);
        float v = (j + sampler->next()) / float(height);

        float phi = 2.0f * M_PI * u;
        float theta = M_PI * v;

        float sinTheta = sinf(theta);
        Vec3 wi;
        wi.x = sinTheta * cosf(phi);
        wi.y = cosf(theta);
        wi.z = sinTheta * sinf(phi);

        reflectedColour = env->sample(u, v);

        pdf = PDF(shadingData, wi);

        if (pdf < EPSILON) 
            pdf = EPSILON;

        return wi;

        /*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
        pdf = SamplingDistributions::uniformSpherePDF(wi);
        reflectedColour = evaluate(shadingData, wi);
        return wi;*/
    }

    template <typename T>
    T clamp(T value, T min, T max) 
    {
        return value < min ? min : (value > max ? max : value);
    }

    Colour evaluate(const Vec3& wi)
    {
        //convert direction to environment map coordinates
        float phi = atan2f(wi.z, wi.x);
        if (phi < 0.0f)
            phi += 2.0f * M_PI;

        float theta = acosf(clamp(wi.y, -1.0f, 1.0f));

        float u = phi / (2.0f * M_PI);
        float v = theta / M_PI;

        return env->sample(u, v);
    }
    float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3())
    {
        // Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
        float phi = atan2f(wi.z, wi.x);
        if (phi < 0.0f)
            phi += 2.0f * M_PI;

        float theta = acosf(clamp(wi.y, -1.0f, 1.0f));

        float u = phi / (2.0f * M_PI);
        float v = theta / M_PI;

        //compute pixel indices
        int i = std::min(std::max(int(u * width), 0), (width - 1));
        int j = std::min(std::max(int(v * height), 0), (height - 1));

        float p_ij = pdf[j * width + i];

        float sinTheta = sinf(theta);

        if (sinTheta < 1e-4f)
            sinTheta = 1e-4f;

        return p_ij * normalizationFactor / sinTheta;
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

        //convert to spherical coordinates 
        float u = (i + sampler->next()) / float(width);
        float v = (j + sampler->next()) / float(height);

        float phi = 2.0f * M_PI * u;
        float theta = M_PI * v;

        float sinTheta = sinf(theta);

        Vec3 wi;
        wi.x = sinTheta * cosf(phi);
        wi.y = cosf(theta);
        wi.z = sinTheta * sinf(phi);


        pdf = PDF(ShadingData(), wi);
        
        if (pdf < EPSILON) 
            pdf = EPSILON;

        return wi;
    }

};