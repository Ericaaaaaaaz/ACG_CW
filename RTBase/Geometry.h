#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;     //o = origin
	Vec3 dir;   //dir = direction
	Vec3 invDir; //invDir = inverse direction
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
		/*invDir = Vec3(
			fabs(dir.x) > 1e-8f ? 1.0f / dir.x : 1e8f,
			fabs(dir.y) > 1e-8f ? 1.0f / dir.y : 1e8f,
			fabs(dir.z) > 1e-8f ? 1.0f / dir.z : 1e8f
		);*/
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

#define EPSILON 0.001f

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
		float denominator = n.dot(r.dir);

		if (fabs(denominator) < 1e-6f)
			return false;

		float numerator = -(n.dot(r.o) + d);

		float hey = numerator / denominator;

		if (hey < EPSILON)
			return false;

		t = hey;

		return true;
		///////   remember to add test code
	}
};



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
		
		if (u < 0 || u > 1.0f)
		{
			return false;
		}
		
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		
		if (v < 0 || (u + v) > 1.0f)
		{
			return false;
		}
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
	// pdf = 1/area(of triangle)   
	// should return a world coordinate
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		// returns random number between 0-1
		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1.0f - sqrtf(r1);
		//float beta = r2 * sqrtf(r2);
		float beta = r2 * sqrtf(r1);
		float gamma = 1 - (alpha + beta);

		Vec3 sampledPos = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		pdf = 1.0f / area;

		return sampledPos;
		//return Vec3(0, 0, 0);
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

	//updates the current bounding box to include the point p
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	// t is the distance between the camera 
	//tmin,i = (min.i - r.o.i) * r.invDir.i
	//tmax,i = (max.i - r.o.i) * r.invDir.i
	bool rayAABB(const Ray& r, float& t)
	{
		float t1 = (min.x - r.o.x) * r.invDir.x;
		float t2 = (max.x - r.o.x) * r.invDir.x;
		float tMinX = std::min(t1, t2);
		float tMaxX = std::max(t1, t2);

		t1 = (min.y - r.o.y) * r.invDir.y;
		t2 = (max.y - r.o.y) * r.invDir.y;
		float tMinY = std::min(t1, t2);
		float tMaxY = std::max(t1, t2);

		t1 = (min.z - r.o.z) * r.invDir.z;
		t2 = (max.z - r.o.z) * r.invDir.z;
		float tMinZ = std::min(t1, t2);
		float tMaxZ = std::max(t1, t2);


		float tEntry = std::max({ tMinX, tMinY, tMinZ });
		float tExit = std::min({ tMaxX, tMaxY, tMaxZ });

		// if the exit is behind the ray, or the exit is before the entry, there's no intersection
		if (tExit < EPSILON)
			return false;  // box is behind us
		if (tEntry > tExit)
			return false;  // no overlap

		// if tEntry is below 0, we clamp it to 0 since we might be inside the box
		if (tEntry < EPSILON)
			tEntry = 0.0f;

		t = tEntry;
		return true;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		//return true;
		float dummyT;
		return rayAABB(r, dummyT);
	}
	// Add code here
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
	
	unsigned int offset; // This can store an offset and number of triangles in a global triangle list for example
	unsigned char num;   // how many traingles in this leaf

	//Triangle* triangles;
	//unsigned char used;

	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Add BVH building code here
		
		//re-initialise the AABB box
		bounds.reset();

		//construct the smallest box that contains all the traingle vertices
		for (auto& tri : inputTriangles)
		{
			//updates the current bounding box to include each vertex position
			bounds.extend(tri.vertices[0].p);
			bounds.extend(tri.vertices[1].p);
			bounds.extend(tri.vertices[2].p);
		}

		//if the triangle count is small(less than or equal to MAXNODE_TRIANGLES)
		//then we dont need to subdivide it further(make it as a leaf in the BVH tree)
		unsigned int numberOfTriangles = (unsigned int)inputTriangles.size();
		
		if (numberOfTriangles <= MAXNODE_TRIANGLES)
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)numberOfTriangles;

			for (auto& tri : inputTriangles)
				outputTriangles.push_back(tri);

			//it wont be splitted, so left and right child are nullptr
			l = nullptr;
			r = nullptr;
			return;
		}

		float parentArea = bounds.area();

		//cost of not splitting(just make this a leaf with triangles): C_leaf = N * C_intersect_primitive
		//N = number of primitives(actual geometric objects like triangles, spheres..)
		//C_intersect_primitive = cost for intersecting a primitive
		float leafCost = (float)numberOfTriangles * TRIANGLE_COST;

		//start with worst-case values
		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestBinBoundary = -1;


		//vectors to store how many triangles fall left and right of the chosen split
		std::vector<int> bestLeftCount, bestRightCount;


		//loop through x,y,z axes(x=0,y=1,z=2)
		for (int axis = 0; axis < 3; axis++)
		{
			//length of the bounding box along x,y,z axes
			float minEdge = bounds.min.coords[axis];
			float maxEdge = bounds.max.coords[axis];
			float lengthOfBoundingBox = maxEdge - minEdge;

			if (lengthOfBoundingBox < 1e-8f)
				continue;

			Bin bins[BUILD_BINS];

			for (auto& tri : inputTriangles)
			{
				// get the centre of the triangle
				float centre = (tri.vertices[0].p.coords[axis] + tri.vertices[1].p.coords[axis] + tri.vertices[2].p.coords[axis]) / 3.0f;

				int binIndex = int(((centre - minEdge) / lengthOfBoundingBox) * float(BUILD_BINS));

				if (binIndex < 0)
					binIndex = 0;

				if (binIndex >= BUILD_BINS)
					binIndex = BUILD_BINS - 1;


				//let the bounding box of the bin to include the triangle's vertices
				bins[binIndex].bounds.extend(tri.vertices[0].p);
				bins[binIndex].bounds.extend(tri.vertices[1].p);
				bins[binIndex].bounds.extend(tri.vertices[2].p);
				bins[binIndex].triangleCount++;


			}

			AABB leftBounds[BUILD_BINS];
			int leftCount[BUILD_BINS];
			AABB  tempBox;
			int   tempCount = 0;
			tempBox.reset();
			for (int i = 0; i < BUILD_BINS; i++)
			{
				tempBox.extend(bins[i].bounds.min);
				tempBox.extend(bins[i].bounds.max);
				tempCount += bins[i].triangleCount;
				leftBounds[i] = tempBox;
				leftCount[i] = tempCount;
			}

			AABB  rightBounds[BUILD_BINS];
			int   rightCount[BUILD_BINS];
			tempBox.reset();
			tempCount = 0;
			for (int i = BUILD_BINS - 1; i >= 0; i--)
			{
				tempBox.extend(bins[i].bounds.min);
				tempBox.extend(bins[i].bounds.max);
				tempCount += bins[i].triangleCount;
				rightBounds[i] = tempBox;
				rightCount[i] = tempCount;
			}

			for (int i = 0; i < BUILD_BINS - 1; i++)
			{
				float leftArea = leftBounds[i].area();
				int   lCount = leftCount[i];
				float rightArea = rightBounds[i + 1].area();
				int   rCount = rightCount[i + 1];

				// SAH cost = TRAVERSE_COST + 
				//            (leftArea / parentArea) * lCount * TRIANGLE_COST +
				//            (rightArea / parentArea) * rCount * TRIANGLE_COST
				float cost = TRAVERSE_COST
					+ (leftArea / parentArea) * (float)lCount * TRIANGLE_COST
					+ (rightArea / parentArea) * (float)rCount * TRIANGLE_COST;

				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBinBoundary = i;
				}
			}
		}


		if (bestCost >= leafCost || bestAxis < 0)
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)numberOfTriangles;
			for (auto& tri : inputTriangles)
				outputTriangles.push_back(tri);
			l = r = nullptr;
			return;
		}


		float minEdge = bounds.min.coords[bestAxis];
		float maxEdge = bounds.max.coords[bestAxis];
		float lengthOfBoundingBox = maxEdge - minEdge;

		std::vector<Triangle> leftTris;
		std::vector<Triangle> rightTris;
		leftTris.reserve(numberOfTriangles);
		rightTris.reserve(numberOfTriangles);
		// boundary in bin => everything with centroid bin <= bestBin => left
		// otherwise => right
		for (auto& tri : inputTriangles)
		{
			float centre = (tri.vertices[0].p.coords[bestAxis] + tri.vertices[1].p.coords[bestAxis] + tri.vertices[2].p.coords[bestAxis]) / 3.0f;
			int binIdx = int(((centre - minEdge) / lengthOfBoundingBox) * float(BUILD_BINS));
			if (binIdx < 0) binIdx = 0;
			if (binIdx >= BUILD_BINS) binIdx = BUILD_BINS - 1;

			if (binIdx <= bestBinBoundary)
				leftTris.push_back(tri);
			else
				rightTris.push_back(tri);
		}

		if (leftTris.empty() || rightTris.empty())
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)numberOfTriangles;
			for (auto& tri : inputTriangles)
				outputTriangles.push_back(tri);
			l = r = nullptr;
			return;
		}

		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTris, outputTriangles);
		r->build(rightTris, outputTriangles);
			
		
	}

	//find the closest intersection in the BVH subtree
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here

		// 1) Check this node’s bounding box first
		float tBox;
		if (!bounds.rayAABB(ray, tBox))
			return;  // Skip if the ray misses this node’s bounds

		// 2) If this node is a leaf, intersect its triangles
		if (!l && !r)
		{
			for (int i = 0; i < num; i++)
			{
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					// If we found a closer intersection, update it
					if (t > 0.0f && t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = offset + i;
						intersection.alpha = 1.0f - (u + v);
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
			return;
		}

		// 3) Otherwise, this node is internal. Check both children’s bounding boxes
		float tLeft = FLT_MAX, tRight = FLT_MAX;
		bool  hitLeft = (l && l->bounds.rayAABB(ray, tLeft));
		bool  hitRight = (r && r->bounds.rayAABB(ray, tRight));

		// If neither child is hit, we’re done
		if (!hitLeft && !hitRight)
			return;

		// If only one child is hit, recurse on just that one
		if (hitLeft && !hitRight)
		{
			l->traverse(ray, triangles, intersection);
			return;
		}
		if (!hitLeft && hitRight)
		{
			r->traverse(ray, triangles, intersection);
			return;
		}

		// 4) If both children are hit, sort them by tLeft/tRight
		//    to traverse the “near” child first.
		if (tLeft < tRight)
		{
			l->traverse(ray, triangles, intersection);

			// If we haven't found a closer intersection than tRight,
			// we still need to test the right child
			if (intersection.t > tRight)
				r->traverse(ray, triangles, intersection);
		}
		else
		{
			r->traverse(ray, triangles, intersection);

			// If we haven't found a closer intersection than tLeft,
			// we still need to test the left child
			if (intersection.t > tLeft)
				l->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX; 
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here

		// 1) If the ray misses this node’s AABB, it’s visible
		float tBox;
		if (!bounds.rayAABB(ray, tBox))
			return true;

		// 2) If this is a leaf node, check each triangle
		if (!l && !r)
		{
			for (int i = 0; i < num; i++)
			{
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t > 0.0f && t < maxT)
						return false;  // Occluded
				}
			}
			return true;  // No triangle occluded the ray
		}

		// 3) Internal node — check both children’s AABBs
		float tLeft = FLT_MAX, tRight = FLT_MAX;
		bool  hitLeft = (l && l->bounds.rayAABB(ray, tLeft));
		bool  hitRight = (r && r->bounds.rayAABB(ray, tRight));

		if (!hitLeft && !hitRight)
			return true;

		if (hitLeft && !hitRight)
			return l->traverseVisible(ray, triangles, maxT);

		if (!hitLeft && hitRight)
			return r->traverseVisible(ray, triangles, maxT);

		// 4) Sort and test children in order of proximity
		if (tLeft < tRight)
		{
			// If left side occludes, return false immediately
			if (!l->traverseVisible(ray, triangles, maxT))
				return false;

			// Otherwise, only test right side if tRight is within range
			if (tRight < maxT)
				return r->traverseVisible(ray, triangles, maxT);
		}
		else
		{
			if (!r->traverseVisible(ray, triangles, maxT))
				return false;

			if (tLeft < maxT)
				return l->traverseVisible(ray, triangles, maxT);
		}

		return true;
		
	}
	
};
