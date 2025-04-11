#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;      // ray origin
	Vec3 dir;    // ray direction
	Vec3 invDir; //inverse direction
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	// r = o + td (o = origin, d = direction, t = gradient)
	// t = -(n dot r.o + d) / (n dot r.dir)
	bool rayIntersect(Ray& r, float& t)
	{

		float denom = Dot(n,r.dir);

		if (fabs(denom) < 1e-6f)
			return false;

		float numerator = -Dot(n, r.o) - d;
		float tCandidate = numerator / denom;

		//if t < 0, the intersection is behind the ray origin
		if (tCandidate < 0.0f)
			return false;

		//otherwise, we have a valid intersection
		t = tCandidate;
		return true;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) 
		{ 
			return false; 
		}
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) 
		{ 
			return false; 
		}
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;

		if (u < -EPSILON || u > 1.0f + EPSILON) 
			return false;

		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;

		if (v < -EPSILON || (u + v) > 1.0f + EPSILON) 
			return false;

		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1.0f - sqrtf(r1);
		//float beta = r2 * sqrtf(r2);
		float beta = r2 * sqrtf(r1);
		float gamma = 1 - (alpha + beta);

		Vec3 sampledPos = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		pdf = 1.0f / area;

		return sampledPos;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	// t is the distance between the camera 
	//tmin,i = (min.i - r.o.i) * r.invDir.i
	//tmax,i = (max.i - r.o.i) * r.invDir.i
	bool rayAABB(const Ray& r, float& tMin,float& tMax)
	{
		float tminx, tmaxx;
		if (fabs(r.dir.x) < 1e-12f)
		{
			if (r.o.x < min.x || r.o.x > max.x) return false;
			tminx = -FLT_MAX;
			tmaxx = +FLT_MAX;
		}
		else
		{
			tminx = (min.x - r.o.x) * r.invDir.x;
			tmaxx = (max.x - r.o.x) * r.invDir.x;
			if (tminx > tmaxx) std::swap(tminx, tmaxx);
		}

		float tminy, tmaxy;
		if (fabs(r.dir.y) < 1e-12f)
		{
			if (r.o.y < min.y || r.o.y > max.y) return false;
			tminy = -FLT_MAX;
			tmaxy = +FLT_MAX;
		}
		else
		{
			tminy = (min.y - r.o.y) * r.invDir.y;
			tmaxy = (max.y - r.o.y) * r.invDir.y;
			if (tminy > tmaxy) std::swap(tminy, tmaxy);
		}

		float tminz, tmaxz;
		if (fabs(r.dir.z) < 1e-12f)
		{
			if (r.o.z < min.z || r.o.z > max.z) return false;
			tminz = -FLT_MAX;
			tmaxz = +FLT_MAX;
		}
		else
		{
			tminz = (min.z - r.o.z) * r.invDir.z;
			tmaxz = (max.z - r.o.z) * r.invDir.z;
			if (tminz > tmaxz) std::swap(tminz, tmaxz);
		}

		tMin = std::max(tminx, std::max(tminy, tminz));
		tMax = std::min(tmaxx, std::min(tmaxy, tmaxz));

		if (tMax < tMin) return false;     // no overlap
		if (tMax < 0.0f)  return false;    // behind the ray
		return true;
	}

	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float b = Dot(l, r.dir);
		float c = Dot(l, l) - radius * radius;
		float discriminant = b * b - c;

		if (discriminant < EPSILON)
			return false;

		float sqrtD = sqrt(discriminant);
		float t0 = -b - sqrtD;
		float t1 = -b + sqrtD;


		if (t0 > EPSILON)
		{
			t = t0;
			return true;
		}
		if (t1 > EPSILON)
		{
			t = t1;
			return true;
		}
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32


//store bin results per axis
//for each axis, we have an array of bins
struct Bin
{
	AABB bounds;
	int triangleCount = 0;
};


class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;

	int start = 0; //start index in global triangle array
	int end = 0;   //end index in global traingle array
	bool leaf = false;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}

	bool isLeaf() const
	{
		return leaf;
	}

	void buildSAH(std::vector<Triangle>& inputTriangles, int startIndex, int endIndex, int depth)
	{
		//re-initialise the AABB box
		bounds.reset();

		//construct the smallest box that contains all the traingle vertices
		for (int i = startIndex; i < endIndex; i++)
		{
			//expand the current bounding box to include each vertex of the triangle
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}

		start = startIndex;
		end = endIndex;
		int numberOfTriangles = endIndex - startIndex;

		//if the triangle count is small(less than or equal to MAXNODE_TRIANGLES)
		//then we dont need to subdivide it further(make it as a leaf in the BVH tree)
		if (numberOfTriangles<= MAXNODE_TRIANGLES || depth > 32)
		{
			leaf = true;
			return;
		}

		AABB centroidBounds;
		centroidBounds.reset();
		for (int i = startIndex; i < endIndex; i++)
		{
			centroidBounds.extend(inputTriangles[i].centre());
		}

		Vec3 csize = centroidBounds.max - centroidBounds.min;

		//if extremely small bounds, make it as a leaf
		if (csize.x < 1e-6f && csize.y < 1e-6f && csize.z < 1e-6f)
		{
			leaf = true;
			return;
		}

		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestBin = -1;

		float parentArea = bounds.area();

		for (int axis = 0; axis < 3; axis++)
		{
			//if the axis extent is extremely small, skip it
			if (csize.coords[axis] < 1e-6f)
				continue;

			//setup bins
			Bin bins[BUILD_BINS];

			for (int b = 0; b < BUILD_BINS; b++)
			{
				bins[b].bounds.reset();
				bins[b].triangleCount = 0;
			}

			float invExtent = 1.0f / csize.coords[axis];


			for (int i = startIndex; i < endIndex; i++)
			{
				//compute fractional offset of the triangle’s centroid along that axis
				float centroidPos = inputTriangles[i].centre().coords[axis] - centroidBounds.min.coords[axis];
				float t = centroidPos * invExtent;
				int b = (int)(t * (BUILD_BINS - 1));
				if (b < 0) b = 0;
				if (b >= BUILD_BINS) b = BUILD_BINS - 1;

				bins[b].triangleCount++;
				bins[b].bounds.extend(inputTriangles[i].vertices[0].p);
				bins[b].bounds.extend(inputTriangles[i].vertices[1].p);
				bins[b].bounds.extend(inputTriangles[i].vertices[2].p);
			}


			AABB leftBounds[BUILD_BINS];
			int leftCount[BUILD_BINS];

			leftBounds[0] = bins[0].bounds;
			leftCount[0] = bins[0].triangleCount;
			for (int i = 1; i < BUILD_BINS; i++)
			{
				leftBounds[i] = leftBounds[i - 1];
				if (bins[i].triangleCount > 0)
				{
					leftBounds[i].extend(bins[i].bounds.min);
					leftBounds[i].extend(bins[i].bounds.max);
				}
				leftCount[i] = leftCount[i - 1] + bins[i].triangleCount;
			}

			AABB rightBounds[BUILD_BINS];
			int rightCount[BUILD_BINS];
			rightBounds[BUILD_BINS - 1] = bins[BUILD_BINS - 1].bounds;
			rightCount[BUILD_BINS - 1] = bins[BUILD_BINS - 1].triangleCount;
			for (int i = BUILD_BINS - 2; i >= 0; i--)
			{
				rightBounds[i] = rightBounds[i + 1];
				if (bins[i].triangleCount > 0)
				{
					rightBounds[i].extend(bins[i].bounds.min);
					rightBounds[i].extend(bins[i].bounds.max);
				}
				rightCount[i] = rightCount[i + 1] + bins[i].triangleCount;
			}

			for (int i = 0; i < BUILD_BINS - 1; i++)
			{
				float leftArea = leftBounds[i].area();
				float rightArea = rightBounds[i + 1].area();
				int c0 = leftCount[i];
				int c1 = rightCount[i + 1];

				// SAH cost =  TRAVERSE_COST +
				//            (leftArea / parentArea) * lCount * TRIANGLE_COST +
				//            (rightArea / parentArea) * rCount * TRIANGLE_COST
				float cost = TRAVERSE_COST +
					(leftArea / parentArea) * (float)c0 * TRIANGLE_COST +
					(rightArea / parentArea) * (float)c1 * TRIANGLE_COST;

				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBin = i;
				}
			}

		}

		//approximate cost if we keep them all in one leaf
		float leafCost = TRIANGLE_COST * (float)numberOfTriangles; 
		if (bestAxis == -1 || bestCost >= leafCost)
		{
			leaf = true;
			return;
		}

		
		float axisMin = centroidBounds.min.coords[bestAxis];
		float axisMax = centroidBounds.max.coords[bestAxis];
		float axisSize = axisMax - axisMin;
		float invExtent = 1.0f / axisSize;
		
		float splitPos = (float)(bestBin + 1) / (float)BUILD_BINS; 
		float cut = axisMin + splitPos * axisSize;


		auto midIt = std::partition(inputTriangles.begin() + startIndex,
			inputTriangles.begin() + endIndex,
			[&](const Triangle& tri)
			{
				return tri.centre().coords[bestAxis] < cut;
			});
		int mid = int(midIt - inputTriangles.begin());

		//if partition fails (all on one side), force leaf
		if (mid == startIndex || mid == endIndex)
		{
			leaf = true;
			return;
		}

		l = new BVHNode();
		r = new BVHNode();
		l->buildSAH(inputTriangles, startIndex, mid, depth + 1);
		r->buildSAH(inputTriangles, mid, endIndex, depth + 1);

	
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		buildSAH(inputTriangles, 0, (int)inputTriangles.size(), 0);

	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float tmin, tmax;
		if (!bounds.rayAABB(ray, tmin, tmax)) 
			return;
		if (tmin > intersection.t) 
			return;

		if (leaf)
		{
			for (int i = start; i < end; i++)
			{
				float tHit, u, v;
				if (triangles[i].rayIntersect(ray, tHit, u, v))
				{
					if (tHit < intersection.t && tHit > 1e-6f)
					{
						intersection.t = tHit;
						intersection.alpha = 1.0f - (u + v);
						intersection.beta = u;
						intersection.gamma = v;
						intersection.ID = i;
					}
				}
			}
			return;
		}

		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		intersection.ID = 0;
		intersection.alpha = intersection.beta = intersection.gamma = 0.0f;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT, const Triangle* ignoreTriangle)
	{
		float tmin;
		float tmax;
		if (!bounds.rayAABB(ray,tmin,tmax )) return true;  
		if (tmin > maxT) return true;      

	
		if (leaf)
		{
			for (int i = start; i < end; i++)
			{
				const Triangle& tri = triangles[i];

				if (&tri == ignoreTriangle)
					continue;

				float tHit, u, v;
				if (tri.rayIntersect(ray, tHit, u, v))
				{
					if (tHit > EPSILON && tHit < maxT)
					{
						
						return false;
					}
				}
			}
			return true;
		}

		bool leftVisible = (l) ? l->traverseVisible(ray, triangles, maxT, ignoreTriangle) : true;
		if (!leftVisible) 
			return false;
		bool rightVisible = (r) ? r->traverseVisible(ray, triangles, maxT, ignoreTriangle) : true;
		return rightVisible;
	}
};