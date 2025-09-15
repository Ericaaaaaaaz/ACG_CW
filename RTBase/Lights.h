#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"
#include <algorithm>
#include <numeric>

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
        //pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		pdf = 1/(4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
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

    std::vector<float> rowPdf;   // size = height
    std::vector<float> rowCdf;   // size = height

    std::vector<float> colPdf;   // size = width * height
    std::vector<float> colCdf;   // size = width * height

    float normalizationFactor;

    EnvironmentMap(Texture* _env)
    {
        env = _env;
        width = env->width;
        height = env->height;

        rowPdf.resize(height);
        rowCdf.resize(height);
        colPdf.resize(width * height);
        colCdf.resize(width * height);

        buildDistributions();

    }

    inline int Idx(int j, int i) const 
    { 
        return j * width + i; 
    
    }

    template <typename T>
    static inline T clamp(T v, T lo, T hi) 
    { 
        return v < lo ? lo : (v > hi ? hi : v); 
    }

    static inline void dirToUv(const Vec3& wi, float& u, float& v, float& theta, float& phi)
    {
        //y-up latitude/longitude
        phi = atan2f(wi.z, wi.x);
        if (phi < 0.0f) phi += 2.0f * M_PI;
        theta = acosf(clamp(wi.y, -1.0f, 1.0f));
        u = phi / (2.0f * M_PI);
        v = theta / M_PI;
    }

    static inline Vec3 uvToDir(float u, float v)
    {
        float phi = 2.0f * M_PI * u;
        float theta = M_PI * v;
        float sT = sinf(theta);
        Vec3 wi;
        wi.x = sT * cosf(phi);
        wi.y = cosf(theta);
        wi.z = sT * sinf(phi);
        return wi;
    }

    void buildDistributions()
    {
        if (width == 0 || height == 0) return;

        //marginal over rows
        std::vector<float> rowWeight(height, 0.0f);

        for (int j = 0; j < height; ++j)
        {
            //theta at row center
            float theta = (float(j) + 0.5f) * (float)M_PI / float(height);
            float sT = sinf(theta);

            //sum luminance over row j
            float sumRowL = 0.0f;
            for (int i = 0; i < width; ++i)
                sumRowL += env->texels[Idx(j, i)].Lum();

            //marginal weight
            rowWeight[j] = sT * sumRowL;
        }

        //normalise rowPdf (fallback to uniform if total is 0)
        float totalRow = std::accumulate(rowWeight.begin(), rowWeight.end(), 0.0f);
        if (totalRow <= 0.0f) 
            totalRow = 1.0f;

        for (int j = 0; j < height; ++j)
            rowPdf[j] = rowWeight[j] / totalRow;

        //row CDF
        float acc = 0.0f;
        for (int j = 0; j < height; ++j) 
        {
            acc += rowPdf[j];
            rowCdf[j] = acc;
        }

        if (rowCdf.back() > 0.0f) 
        {
            float invLast = 1.0f / rowCdf.back();
            for (float& x : rowCdf) x *= invLast;
        }

        //conditional over columns per row (L / sumRowL)
        for (int j = 0; j < height; ++j)
        {
            //sum luminance on row j
            float sumRowL = 0.0f;
            for (int i = 0; i < width; ++i)
                sumRowL += env->texels[Idx(j, i)].Lum();

            if (sumRowL <= 0.0f) sumRowL = 1.0f; //uniform fallback on empty rows

            //normalised per-row pdf and cdf
            float rowAcc = 0.0f;
            for (int i = 0; i < width; ++i)
            {
                float lij = env->texels[Idx(j, i)].Lum();
                colPdf[Idx(j, i)] = lij / sumRowL;

                rowAcc += colPdf[Idx(j, i)];
                colCdf[Idx(j, i)] = rowAcc;
            }

            //normalise CDF of row j to exactly 1
            float last = colCdf[Idx(j, width - 1)];
            if (last > 0.0f)
            {
                float inv = 1.0f / last;
                for (int i = 0; i < width; ++i)
                    colCdf[Idx(j, i)] *= inv;
            }
        }
    }



    Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
    {
        // Assignment: Update this code to importance sampling lighting based on luminance of each pixel
        //sample row (v) from marginal CDF
        float r1 = sampler->next();
        int j = int(std::lower_bound(rowCdf.begin(), rowCdf.end(), r1) - rowCdf.begin());
        j = clamp(j, 0, height - 1);

        //sample column (u) from conditional CDF of the chosen row
        float r2 = sampler->next();
        const float* cdfRow = &colCdf[Idx(j, 0)];
        int i = int(std::lower_bound(cdfRow, cdfRow + width, r2) - cdfRow);
        i = clamp(i, 0, width - 1);

        float u = (i + sampler->next()) / float(width);
        float v = (j + sampler->next()) / float(height);

        Vec3 wi = uvToDir(u, v);
        emittedColour = env->sample(u, v);

        float theta = M_PI * v;
        float sT = sinf(theta);
        sT = (sT < 1e-4f) ? 1e-4f : sT;

        //probability of choosing pixel (i,j)
        float Pij = rowPdf[j] * colPdf[Idx(j, i)];
        float p_uv = Pij * float(width * height);           
        pdf = p_uv / (2.0f * float(M_PI) * float(M_PI) * sT);

        if (pdf < EPSILON) pdf = EPSILON;
        return wi;
    }


    Colour evaluate(const Vec3& wi)
    {
        float u, v, theta, phi;
        dirToUv(wi, u, v, theta, phi);
        return env->sample(u, v);
    }
    float PDF(const ShadingData& shadingData, const Vec3& wi, const Vec3& targetPos = Vec3())
    {
        // Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
        float u, v, theta, phi;
        dirToUv(wi, u, v, theta, phi);

        int i = clamp(int(u * width), 0, width - 1);
        int j = clamp(int(v * height), 0, height - 1);

        float sT = sinf(theta);
        sT = (sT < 1e-4f) ? 1e-4f : sT;

        float Pij = rowPdf[j] * colPdf[Idx(j, i)];
        float p_uv = Pij * float(width * height);
        return p_uv / (2.0f * float(M_PI) * float(M_PI) * sT);
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
        float total = 0.0f;
        float dTheta = float(M_PI) / float(height);
        float dPhi = 2.0f * float(M_PI) / float(width);

        for (int j = 0; j < height; ++j) 
        {
            float theta = (j + 0.5f) * dTheta;
            float sT = sinf(theta);
            for (int i = 0; i < width; ++i)
                total += env->texels[Idx(j, i)].Lum() * sT * (dTheta * dPhi);
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
        Colour dummy;
        return sample(ShadingData(), sampler, dummy, pdf);
    }

};