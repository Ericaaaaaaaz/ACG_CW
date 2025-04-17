#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

inline float clamp(float x, float minVal, float maxVal)
{
	if (x < minVal)
	{
		return minVal;
	}
	else if (x > maxVal)
	{
		return maxVal;
	}
	else
	{
		return x;
	}
}


class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		float R0 = (iorInt - iorExt) / (iorInt + iorExt);
		R0 = R0 * R0;
		float c = 1.0f - clamp(cosTheta, 0.0f, 1.0f);
		float c2 = c * c;
		return R0 + (1.0f - R0) * c2 * c2 * c;
		//return 1.0f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		return Colour(1.0f, 1.0f, 1.0f);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = fabs(wi.z); // z-up system
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));


		if (cosTheta == 0.0f)
			return 1e10f;
		float tanTheta = sinTheta / cosTheta;
		float a = 1.0f / (alpha * tanTheta);

		if (a >= 1.6f)
			return 0.0f;
		return (1.0f - 1.259f * a + 0.396f * a * a) / (3.535f * a + 2.181f * a * a);

		//return 1.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		//return 1.0f;
		return 1.0f / (1.0f + ShadingHelper::lambdaGGX(wi, alpha) + ShadingHelper::lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		//return 1.0f;
		float cos_theta = h.z;
		if (cos_theta <= 0)
			return 0.0f;
		float tan2_theta = (1.0f - cos_theta * cos_theta) / (cos_theta * cos_theta);
		float alpha2 = alpha * alpha;
		float factor = tan2_theta / alpha2;
		float cos4_theta = cos_theta * cos_theta * cos_theta * cos_theta;
		return alpha2 / (M_PI * cos4_theta * (1.0f + factor) * (1.0f + factor));
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(
			sampler->next(), sampler->next());

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		pdf = wiLocal.z / M_PI;                       
		if (pdf < EPSILON) 
		{ 
			reflectedColour = Colour(0.0f,0.0f,0.0f); 
			return wi; 
		}

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		//return 1.0f;
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return (wiLocal.z > 0.0f) ? wiLocal.z / M_PI : 0.0f;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wi = Vec3(-wo.x, -wo.y, wo.z);
		pdf = 1.0f;
		float cosTheta = std::abs(wi.z);
		//Vec3 worldWi = shadingData.frame.toWorld(wi);
		wi = shadingData.frame.toWorld(wi);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / cosTheta;
		return wi;

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		Vec3 idealReflection = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		float reflectionDot = Dot(idealReflection.normalize(), wiLocal.normalize());

		if (reflectionDot > 0.999f)
		{
			return albedo->sample(shadingData.tu, shadingData.tv) / std::abs(wiLocal.z);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 1.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		float u1 = sampler->next();
		float u2 = sampler->next();

		float phi = 2.0f * M_PI * u1;
		float theta = atanf(alpha * sqrtf(u2 / (1.0f - u2)));

		Vec3 h = Vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));

		Vec3 wi = h * 2.0f * Dot(wo, h) - wo;

		if (wi.z <= 0.0f) 
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return shadingData.frame.toWorld(Vec3(0, 0, 1));
		}


		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wi, wo, alpha);
		Colour F = ShadingHelper::fresnelConductor(Dot(wi, h), eta, k);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F * D * G / (4.0f * wo.z * wi.z);


		pdf = D * h.z / (4.0f * Dot(wo, h));

		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		bool entering = (woLocal.z > 0.0f);
		float etaI = extIOR, etaT = intIOR;
		float cosThetaI = woLocal.z;

		if (!entering)
		{
			std::swap(etaI, etaT);
			//make cosThetaI positive for the fresnel function
			cosThetaI = -cosThetaI;
		}


		float F = ShadingHelper::fresnelDielectric(clamp(cosThetaI, 0.0f, 1.0f), etaI, etaT);

		//randomly pick reflection vs refraction
		float randVal = sampler->next();
		Vec3 wiLocal;
		if (randVal < F)
		{
			wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);

			Colour alb = albedo->sample(shadingData.tu, shadingData.tv);
			reflectedColour = alb * F;

			pdf = F;
		}
		else
		{
			//refraction
			float eta = etaI / etaT;
			float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
			float sinThetaT2 = eta * eta * sinThetaI2;

			if (sinThetaT2 >= 1.0f)
			{
				//total internal reflection => reflect
				wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);

				Colour alb = albedo->sample(shadingData.tu, shadingData.tv);
				reflectedColour = alb;
				pdf = 1.0f;
			}
			else
			{
				float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT2));
				if (entering)
					cosThetaT = -cosThetaT;


				wiLocal.x = eta * (-woLocal.x);
				wiLocal.y = eta * (-woLocal.y);
				wiLocal.z = eta * woLocal.z - (cosThetaT - eta * cosThetaI);


				Colour alb = albedo->sample(shadingData.tu, shadingData.tv);
				reflectedColour = alb * (1.0f - F);

				pdf = 1.0f-F;
			}
		}

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		bool entering = (woLocal.z > 0.0f);
		float etaI = extIOR, etaT = intIOR;

		if (!entering)
			std::swap(etaI, etaT);


		Vec3 idealReflection = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		idealReflection = shadingData.frame.toWorld(idealReflection);
		float reflectionDot = Dot(idealReflection, wi);

		if (reflectionDot > 0.999f) 
		{
			float F = ShadingHelper::fresnelDielectric(fabs(woLocal.z), etaI, etaT);
			return albedo->sample(shadingData.tu, shadingData.tv) * F;
		}


		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);

	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};