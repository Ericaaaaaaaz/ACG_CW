#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include <unordered_map>

class Camera
{
public:
	Matrix projectionMatrix;
	Matrix inverseProjectionMatrix;
	Matrix camera;
	Matrix cameraToView;
	float width = 0;
	float height = 0;
	Vec3 origin;
	Vec3 viewDirection;
	float Afilm;
	void init(Matrix ProjectionMatrix, int screenwidth, int screenheight)
	{
		projectionMatrix = ProjectionMatrix;
		inverseProjectionMatrix = ProjectionMatrix.invert();
		width = (float)screenwidth;
		height = (float)screenheight;
		float Wlens = (2.0f / ProjectionMatrix.a[1][1]);
		float aspect = ProjectionMatrix.a[0][0] / ProjectionMatrix.a[1][1];
		float Hlens = Wlens * aspect;
		Afilm = Wlens * Hlens;
	}
	void updateView(Matrix V)
	{
		camera = V;
		cameraToView = V.invert();
		origin = camera.mulPoint(Vec3(0, 0, 0));
		viewDirection = inverseProjectionMatrix.mulPointAndPerspectiveDivide(Vec3(0, 0, 1));
		viewDirection = camera.mulVec(viewDirection);
		viewDirection = viewDirection.normalize();
	}
	// Add code here
	Ray generateRay(float x, float y)
	{
		float xprime = x / width;
		float yprime = 1.0f - (y / height);
		xprime = (xprime * 2.0f) - 1.0f;
		yprime = (yprime * 2.0f) - 1.0f;
		Vec3 dir(xprime, yprime, 1.0f);
		dir = inverseProjectionMatrix.mulPointAndPerspectiveDivide(dir);
		dir = camera.mulVec(dir);
		dir = dir.normalize();
		return Ray(origin, dir);

	}
	bool projectOntoCamera(const Vec3& p, float& x, float& y)
	{
		Vec3 pview = cameraToView.mulPoint(p);
		Vec3 pproj = projectionMatrix.mulPointAndPerspectiveDivide(pview);
		x = (pproj.x + 1.0f) * 0.5f;
		y = (pproj.y + 1.0f) * 0.5f;
		if (x < 0 || x > 1.0f || y < 0 || y > 1.0f)
		{
			return false;
		}
		x = x * width;
		y = 1.0f - y;
		y = y * height;
		return true;
	}
};

static void computeSmoothNormals(std::vector<Triangle>& tris, float posEps = 1e-6f)
{
	struct Key 
	{
		int64_t x, y, z;
		bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
	};
	struct H 
	{
		size_t operator()(const Key& k) const 
		{
			//64-bit mix
			size_t h = 1469598103934665603ull;
			auto mix = [&](int64_t v) 
				{ 
					h ^= (size_t)v; h *= 1099511628211ull; 
				};
			mix(k.x); mix(k.y); mix(k.z); 
			return h;
		}
	};
	auto quant = [&](const Vec3& p)->Key 
		{
			const double s = 1.0 / posEps;
			return { (int64_t)llround(p.x * s), (int64_t)llround(p.y * s), (int64_t)llround(p.z * s) };
		};

	//accumulate face normals per position
	std::unordered_map<Key, Vec3, H> acc;
	std::unordered_map<Key, int, H> cnt;

	for (const Triangle& t : tris) 
	{
		Vec3 e1 = t.vertices[1].p - t.vertices[0].p;
		Vec3 e2 = t.vertices[2].p - t.vertices[0].p;
		Vec3 faceN = e1.cross(e2); 

		for (int i = 0; i < 3; ++i) 
		{
			Key k = quant(t.vertices[i].p);
			acc[k] = acc[k] + faceN;
			cnt[k] += 1;
		}
	}

	//write back normalized smooth normals
	for (Triangle& t : tris)
	{
		for (int i = 0; i < 3; ++i) 
		{
			Key k = quant(t.vertices[i].p);
			Vec3 n = acc[k];
			if (n.lengthSq() > 0.0f) 
				n = n.normalize();

			if (Dot(n, t.n) < 0.0f) 
				n = -n;

			t.vertices[i].normal = n;
		}

		//recompute geometric normal as well (keeps t.n consistent)
		Vec3 e1 = t.vertices[1].p - t.vertices[0].p;
		Vec3 e2 = t.vertices[2].p - t.vertices[0].p;
		t.n = e1.cross(e2).normalize();
	}
}

class Scene
{
public:
	std::vector<Triangle> triangles;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;
	Light* background = NULL;
	BVHNode* bvh = NULL;
	Camera camera;
	AABB bounds;


	void build()
	{
		computeSmoothNormals(triangles);

		// Add BVH building code here
		bvh = new BVHNode();
		bvh->build(triangles);

		// Do not touch the code below this line!
		// Build light list
		for (int i = 0; i < triangles.size(); i++)
		{
			if (materials[triangles[i].materialIndex]->isLight())
			{
				AreaLight* light = new AreaLight();
				light->triangle = &triangles[i];
				light->emission = materials[triangles[i].materialIndex]->emission;
				lights.push_back(light);
			}
		}
	}
	IntersectionData traverse(const Ray& ray)
	{
		return bvh->traverse(ray,triangles);
	}
	Light* sampleLight(Sampler* sampler, float& pmf)
	{
		//r in the range[0, 1)
		float r1 = sampler->next();
		pmf = 1.0f / (float)lights.size();
		return lights[std::min((int)(r1 * lights.size()), (int)(lights.size() - 1))];
	}
	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background)
	{
		for (int i = 0; i < meshTriangles.size(); i++)
		{
			triangles.push_back(meshTriangles[i]);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}
		for (int i = 0; i < meshMaterials.size(); i++)
		{
			materials.push_back(meshMaterials[i]);
		}
		background = _background;
		if (background->totalIntegratedPower() > 0)
		{
			lights.push_back(background);
		}
	}
	bool visible(const Vec3& p1, const Vec3& p2, const Triangle* ignoreTriangle) const
	{
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length();
		dir = dir.normalize();
		ray.init(p1 + (dir * EPSILON), dir);
		return bvh->traverseVisible(ray, triangles, maxT, ignoreTriangle);
	}
	Colour emit(Triangle* light, ShadingData shadingData, Vec3 wi)
	{

		return materials[light->materialIndex]->emit(shadingData, wi);
	}
	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX)
		{
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.sNormal) < 0)
				{
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0)
				{
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		}
		else
		{
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};