#include "../include/FreeImage.h"
#include <string>
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <thread>
#define _USE_MATH_DEFINES
#include <math.h>
#include "../include/glm/gtc/matrix_transform.hpp"
#include "../include/glm/gtc/constants.hpp"

#include "Camera.h"
#include "Ray.h"
#include "Intersection.h"
#include "Scene.h"
#include <mutex>
#include <atomic>
#include <sstream>
#include <random>

static inline float RandFloat()
{
	thread_local std::mt19937 rng(std::random_device{}());
	thread_local std::uniform_real_distribution<float> dist(0.0f, 0.99999f);
	return dist(rng);
}

static inline float RandFloatOne()
{
	thread_local std::mt19937 rng(std::random_device{}());
	thread_local std::uniform_real_distribution<float> dist(0.0f, 1.0f);
	return dist(rng);
}

std::mutex g_logMutex;
std::atomic<int> g_rowsCompleted = 0;

void LogLine(const std::string& message)
{
	std::lock_guard<std::mutex> lock(g_logMutex);
	std::cout << message << std::endl;
}

int bounces = 0;
int maxDepth = 5;
glm::mat4 modelStack = glm::mat4(1.0f);
glm::mat4 prevMatrix = glm::mat4(1.0f);
glm::mat4 invModelStack = glm::mat4(1.0f);
glm::mat4 prevInvMatrix = glm::mat4(1.0f);
glm::mat4 normalMatrix = glm::mat4(1.0f);
glm::mat4 scaleMatrix = glm::mat4(1.0f);
std::vector<glm::mat4> transformStack = { glm::mat4(1.0f) };
float constant = 1;
float linear = 0;
float quadratic = 0;

struct AABB
{
	glm::vec3 min = glm::vec3(std::numeric_limits<float>::max());
	glm::vec3 max = glm::vec3(-std::numeric_limits<float>::max());

	void Expand(const glm::vec3& p)
	{
		min = glm::min(min, p);
		max = glm::max(max, p);
	}

	void Expand(const AABB& other)
	{
		min = glm::min(min, other.min);
		max = glm::max(max, other.max);
	}

	glm::vec3 Centroid() const
	{
		return 0.5f * (min + max);
	}

	glm::vec3 Extent() const
	{
		return max - min;
	}
};

AABB ComputeTriangleAABB(const Triangle* t)
{
	AABB box;
	box.Expand(t->vertex0);
	box.Expand(t->vertex1);
	box.Expand(t->vertex2);

	glm::vec3 eps(1e-4f);
	box.min -= eps;
	box.max += eps;

	return box;
}

AABB ComputeSphereAABB(const Sphere* s)
{
	AABB box;

	glm::vec3 localCenter = s->center;
	glm::vec3 worldCenter = glm::vec3(s->transform * glm::vec4(localCenter, 1.0f));

	glm::vec3 localRadius(s->radius, s->radius, s->radius);

	glm::vec3 worldX = glm::vec3(s->transform * glm::vec4(localRadius.x, 0.0f, 0.0f, 0.0f));
	glm::vec3 worldY = glm::vec3(s->transform * glm::vec4(0.0f, localRadius.y, 0.0f, 0.0f));
	glm::vec3 worldZ = glm::vec3(s->transform * glm::vec4(0.0f, 0.0f, localRadius.z, 0.0f));

	glm::vec3 extent(
		std::abs(worldX.x) + std::abs(worldY.x) + std::abs(worldZ.x),
		std::abs(worldX.y) + std::abs(worldY.y) + std::abs(worldZ.y),
		std::abs(worldX.z) + std::abs(worldY.z) + std::abs(worldZ.z));

	box.min = worldCenter - extent;
	box.max = worldCenter + extent;

	return box;
}

AABB ComputeSceneBounds(Scene* scene)
{
	AABB sceneBox;

	for (const Sphere* s : scene->GetSpheres())
	{
		sceneBox.Expand(ComputeSphereAABB(s));
	}

	for (const Triangle* t : scene->GetTriangles())
	{
		sceneBox.Expand(ComputeTriangleAABB(t));
	}

	glm::vec3 eps(1e-4f);
	sceneBox.min -= eps;
	sceneBox.max += eps;

	return sceneBox;
}

bool IntersectAABB(const Ray& ray, const AABB& box, float& tEnter, float& tExit)
{
	float tMin = 0.0f;
	float tMax = std::numeric_limits<float>::max();

	for (int axis = 0; axis < 3; ++axis)
	{
		float o = ray.origin[axis];
		float d = ray.direction[axis];
		float bmin = box.min[axis];
		float bmax = box.max[axis];

		if (std::abs(d) < 1e-8f)
		{
			if (o < bmin || o > bmax)
			{
				return false;
			}
			continue;
		}

		float invD = 1.0f / d;
		float t0 = (bmin - o) * invD;
		float t1 = (bmax - o) * invD;

		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		tMin = std::max(tMin, t0);
		tMax = std::min(tMax, t1);

		if (tMax < tMin)
		{
			return false;
		}
	}

	tEnter = tMin;
	tExit = tMax;
	return true;
}

enum class BVHPrimitiveType
{
	Sphere,
	Triangle
};

struct BVHPrimitive
{
	BVHPrimitiveType type;
	int index = -1;
	AABB bounds;
	glm::vec3 centroid = glm::vec3(0.0f);
};

struct BVHNode
{
	AABB bounds;
	int left = -1;
	int right = -1;
	int start = 0;
	int count = 0;

	bool IsLeaf() const
	{
		return left == -1 && right == -1;
	}
};

struct UniformGrid
{
	AABB bounds;
	std::vector<BVHPrimitive> primitives;
	std::vector<BVHNode> nodes;
	int root = -1;
};

static const int BVH_LEAF_SIZE = 4;

int BuildBVHRecursive(UniformGrid& bvh, int start, int end)
{
	BVHNode node;
	AABB nodeBounds;
	AABB centroidBounds;

	for (int i = start; i < end; ++i)
	{
		nodeBounds.Expand(bvh.primitives[i].bounds);
		centroidBounds.Expand(bvh.primitives[i].centroid);
	}

	node.bounds = nodeBounds;

	int count = end - start;
	int nodeIndex = static_cast<int>(bvh.nodes.size());
	bvh.nodes.push_back(node);

	if (count <= BVH_LEAF_SIZE)
	{
		bvh.nodes[nodeIndex].start = start;
		bvh.nodes[nodeIndex].count = count;
		return nodeIndex;
	}

	glm::vec3 extent = centroidBounds.Extent();
	int axis = 0;
	if (extent.y > extent.x && extent.y >= extent.z)
	{
		axis = 1;
	}
	else if (extent.z > extent.x && extent.z >= extent.y)
	{
		axis = 2;
	}

	if (extent[axis] < 1e-8f)
	{
		bvh.nodes[nodeIndex].start = start;
		bvh.nodes[nodeIndex].count = count;
		return nodeIndex;
	}

	int mid = start + count / 2;

	std::nth_element(
		bvh.primitives.begin() + start,
		bvh.primitives.begin() + mid,
		bvh.primitives.begin() + end,
		[axis](const BVHPrimitive& a, const BVHPrimitive& b)
		{
			return a.centroid[axis] < b.centroid[axis];
		});

	int leftChild = BuildBVHRecursive(bvh, start, mid);
	int rightChild = BuildBVHRecursive(bvh, mid, end);

	bvh.nodes[nodeIndex].left = leftChild;
	bvh.nodes[nodeIndex].right = rightChild;
	bvh.nodes[nodeIndex].count = 0;

	return nodeIndex;
}

void BuildUniformGrid(UniformGrid& grid, Scene* scene)
{
	grid.primitives.clear();
	grid.nodes.clear();
	grid.root = -1;

	std::vector<Sphere*>& spheres = scene->GetSpheresRef();
	std::vector<Triangle*>& triangles = scene->GetTriangles();

	grid.primitives.reserve(spheres.size() + triangles.size());

	for (int i = 0; i < static_cast<int>(spheres.size()); ++i)
	{
		BVHPrimitive prim;
		prim.type = BVHPrimitiveType::Sphere;
		prim.index = i;
		prim.bounds = ComputeSphereAABB(spheres[i]);
		prim.centroid = prim.bounds.Centroid();
		grid.primitives.push_back(prim);
	}

	for (int i = 0; i < static_cast<int>(triangles.size()); ++i)
	{
		BVHPrimitive prim;
		prim.type = BVHPrimitiveType::Triangle;
		prim.index = i;
		prim.bounds = ComputeTriangleAABB(triangles[i]);
		prim.centroid = prim.bounds.Centroid();
		grid.primitives.push_back(prim);
	}

	if (grid.primitives.empty())
	{
		return;
	}

	grid.bounds = ComputeSceneBounds(scene);
	grid.root = BuildBVHRecursive(grid, 0, static_cast<int>(grid.primitives.size()));
}

float CheckSphereIntersection(Sphere* sphere, Ray& ray)
{
	glm::vec3 oc = ray.origin - sphere->center;

	float a = glm::dot(ray.direction, ray.direction);
	float b = 2.0f * glm::dot(ray.direction, oc);
	float c = glm::dot(oc, oc) - sphere->radius * sphere->radius;

	float discriminant = b * b - 4.0f * a * c;

	if (discriminant < 0.0f)
	{
		return INFINITY;
	}

	float sqrtDiscriminant = glm::sqrt(discriminant);
	float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
	float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

	if (t1 > 0.0f)
	{
		return t1;
	}

	if (t2 > 0.0f)
	{
		return t2;
	}

	return INFINITY;
}

glm::vec3 CheckTriangleIntersection(Triangle* triangle, Ray& ray)
{
	const float epsilon = 1e-6f;

	glm::vec3 e1 = triangle->vertex1 - triangle->vertex0;
	glm::vec3 e2 = triangle->vertex2 - triangle->vertex0;

	glm::vec3 pvec = glm::cross(ray.direction, e2);
	float det = glm::dot(e1, pvec);

	if (fabs(det) < epsilon)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	float invDet = 1.0f / det;
	glm::vec3 tvec = ray.origin - triangle->vertex0;

	float beta = glm::dot(tvec, pvec) * invDet;
	if (beta < 0.0f || beta > 1.0f)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	glm::vec3 qvec = glm::cross(tvec, e1);
	float gamma = glm::dot(ray.direction, qvec) * invDet;
	if (gamma < 0.0f || beta + gamma > 1.0f)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	float t = glm::dot(e2, qvec) * invDet;
	if (t <= epsilon)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	return glm::vec3(t, beta, gamma);
}

bool TraverseUniformGrid(
	UniformGrid& grid,
	Scene* scene,
	Ray& ray,
	Intersection& closestHit)
{
	if (grid.root == -1)
	{
		return false;
	}

	float rootEnter, rootExit;
	if (!IntersectAABB(ray, grid.nodes[grid.root].bounds, rootEnter, rootExit))
	{
		return false;
	}

	std::vector<Sphere*>& spheres = scene->GetSpheresRef();
	std::vector<Triangle*>& triangles = scene->GetTriangles();

	bool hitAnything = false;
	float bestT = std::numeric_limits<float>::infinity();

	int stack[128];
	int stackTop = 0;
	stack[stackTop++] = grid.root;

	while (stackTop > 0)
	{
		int nodeIndex = stack[--stackTop];
		const BVHNode& node = grid.nodes[nodeIndex];

		float nodeEnter, nodeExit;
		if (!IntersectAABB(ray, node.bounds, nodeEnter, nodeExit))
		{
			continue;
		}

		if (nodeEnter > bestT)
		{
			continue;
		}

		if (node.IsLeaf())
		{
			for (int i = 0; i < node.count; ++i)
			{
				const BVHPrimitive& prim = grid.primitives[node.start + i];

				if (prim.type == BVHPrimitiveType::Sphere)
				{
					Sphere* sphere = spheres[prim.index];
					glm::mat4 invTransform = glm::inverse(sphere->transform);

					Ray transformedRay(
						glm::vec3(invTransform * glm::vec4(ray.origin, 1.0f)),
						glm::vec3(invTransform * glm::vec4(ray.direction, 0.0f)));

					float sphereTLocal = CheckSphereIntersection(sphere, transformedRay);
					if (sphereTLocal < INFINITY)
					{
						glm::vec3 localHitPoint = transformedRay.origin + transformedRay.direction * sphereTLocal;
						glm::vec3 worldHitPoint = glm::vec3(sphere->transform * glm::vec4(localHitPoint, 1.0f));
						float worldT = glm::dot(worldHitPoint - ray.origin, ray.direction);

						if (worldT > 0.0f && worldT < bestT)
						{
							bestT = worldT;
							closestHit.t = worldT;
							closestHit.sphereIndex = prim.index;
							closestHit.triangleIndex = -1;
							closestHit.hitObjectIsSphere = true;
							hitAnything = true;
						}
					}
				}
				else
				{
					Triangle* triangle = triangles[prim.index];
					glm::vec3 tBetaGamma = CheckTriangleIntersection(triangle, ray);
					float triT = tBetaGamma.x;

					if (triT > 0.0f && triT < bestT && triT < INFINITY)
					{
						bestT = triT;
						closestHit.t = triT;
						closestHit.sphereIndex = -1;
						closestHit.triangleIndex = prim.index;
						closestHit.hitObjectIsSphere = false;
						hitAnything = true;
					}
				}
			}
		}
		else
		{
			const BVHNode& leftNode = grid.nodes[node.left];
			const BVHNode& rightNode = grid.nodes[node.right];

			float leftEnter, leftExit;
			float rightEnter, rightExit;

			bool hitLeft = IntersectAABB(ray, leftNode.bounds, leftEnter, leftExit) && leftEnter <= bestT;
			bool hitRight = IntersectAABB(ray, rightNode.bounds, rightEnter, rightExit) && rightEnter <= bestT;

			if (hitLeft && hitRight)
			{
				if (leftEnter < rightEnter)
				{
					stack[stackTop++] = node.right;
					stack[stackTop++] = node.left;
				}
				else
				{
					stack[stackTop++] = node.left;
					stack[stackTop++] = node.right;
				}
			}
			else if (hitLeft)
			{
				stack[stackTop++] = node.left;
			}
			else if (hitRight)
			{
				stack[stackTop++] = node.right;
			}
		}
	}

	return hitAnything;
}

Ray ShootRay(const Camera& cam, int i, int j, const int width, const int height)
{
	glm::vec3 w = glm::normalize(cam.getEyePos() - cam.getCenter());
	glm::vec3 u = glm::normalize(glm::cross(cam.getUp(), w));
	glm::vec3 v = glm::cross(w, u);

	float fovY = cam.getFovY();
	float aspect = static_cast<float>(width) / static_cast<float>(height);
	float fovX = 2.0f * atan(aspect * tan(fovY * 0.5f));

	float alpha = tan(fovX * 0.5f) * ((2.0f * (static_cast<float>(i) + 0.5f) / static_cast<float>(width)) - 1.0f);
	float beta = tan(fovY * 0.5f) * (1.0f - (2.0f * (static_cast<float>(j) + 0.5f) / static_cast<float>(height)));

	glm::vec3 direction = glm::normalize(alpha * u + beta * v - w);
	return Ray(cam.getEyePos(), direction);
}

Intersection FindIntersection(UniformGrid* grid, Scene* scene, Ray& ray, bool isLightSample)
{
	float minDist = INFINITY;
	Sphere* hitSphere = NULL;
	Triangle* hitTri = NULL;
	bool didHit = false;
	float worldt = minDist;
	float t = minDist;
	float tSphere = minDist;
	float tTriangle = minDist;
	glm::vec3 hitObjectDiffuse = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectSpecular = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectEmission = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectAmbient = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectNormal = glm::vec3(0, 0, 0);
	glm::vec3 sphereLocalNormal = glm::vec3(0, 0, 0);
	glm::vec3 sphereWorldNormal = glm::vec3(0, 0, 0);
	float hitObjectShininess = 1.0f;
	glm::vec3 tBetaGamma2;
	glm::vec3 intersectionPoint(INFINITY, INFINITY, INFINITY);
	bool hitObjectIsSphere = false;
	float hitObjectIOR = 1.0f;
	bool hitHasTexture = false;
	texture* hitObjectTexture = NULL;
	bool isAreaLight = false;
	std::vector<Sphere*>& sceneSpheres = scene->GetSpheres();


	Intersection intersection;
	didHit = TraverseUniformGrid(*grid, scene, ray, intersection);

	if (didHit)
	{
		if (intersection.hitObjectIsSphere)
		{
			/*if (isLightSample)
				return Intersection(
					false,
					intersectionPoint,
					hitObjectDiffuse,
					hitObjectSpecular,
					hitObjectEmission,
					hitObjectShininess,
					hitObjectAmbient,
					hitObjectNormal,
					glm::vec3(0, 0, 0),
					1.0f,
					minDist,
					false,
					hitHasTexture,
					NULL
				);*/

			Sphere* hitSphere = sceneSpheres[intersection.sphereIndex];

			glm::mat4 invTransform = glm::inverse(hitSphere->transform);

			Ray transformedRay(
				glm::vec3(invTransform * glm::vec4(ray.origin, 1.0f)),
				glm::vec3(invTransform * glm::vec4(ray.direction, 0.0f)));

			float tSphere = CheckSphereIntersection(hitSphere, transformedRay);

			glm::vec3 localHitPoint = transformedRay.origin + transformedRay.direction * tSphere;
			glm::vec3 localNormal = glm::normalize(localHitPoint - hitSphere->center);

			glm::vec3 intersectionPoint = glm::vec3(hitSphere->transform * glm::vec4(localHitPoint, 1.0f));
			glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(hitSphere->transform)));
			glm::vec3 hitObjectNormal = glm::normalize(normalMat * localNormal);

			return Intersection(
				true,
				intersectionPoint,
				hitSphere->diffuse,
				hitSphere->specular,
				hitSphere->emission,
				hitSphere->shininess,
				hitSphere->ambient,
				hitObjectNormal,
				glm::vec3(hitSphere->transform * glm::vec4(hitSphere->center, 1.0f)),
				hitSphere->ior,
				intersection.t,
				false,
				hitSphere->matTexture != NULL,
				hitSphere->matTexture
			);

			return intersection;

		}
		else if (intersection.triangleIndex != -1)
		{
			std::vector<Triangle*>& sceneTriangles = scene->GetTriangles();
			Triangle* hitTri = sceneTriangles[intersection.triangleIndex];

			/*if (isLightSample && !hitTri->isLight)
				return Intersection(
					false,
					intersectionPoint,
					hitObjectDiffuse,
					hitObjectSpecular,
					hitObjectEmission,
					hitObjectShininess,
					hitObjectAmbient,
					hitObjectNormal,
					glm::vec3(0, 0, 0),
					1.0f,
					minDist,
					false,
					hitHasTexture,
					NULL
				);*/

				//	glm::vec3 tBetaGamma = CheckTriangleIntersection(hitTri, ray);
				//	glm::vec3 tBetaGamma = intersection.t;
			float tTriangle = intersection.t;
			glm::vec3 intersectionPoint = ray.origin + ray.direction * tTriangle;

			return Intersection(
				true,
				intersectionPoint,
				hitTri->diffuse,
				hitTri->specular,
				hitTri->emission,
				hitTri->shininess,
				hitTri->ambient,
				hitTri->normal,
				glm::vec3(0.0f),
				1.0f,
				tTriangle,
				hitTri->isLight,
				false,
				NULL
			);

			;
		}
	}

	return Intersection(
		false,
		intersectionPoint,
		hitObjectDiffuse,
		hitObjectSpecular,
		hitObjectEmission,
		hitObjectShininess,
		hitObjectAmbient,
		hitObjectNormal,
		glm::vec3(0, 0, 0),
		1.0f,
		minDist,
		false,
		hitHasTexture,
		NULL
	);
}

glm::vec3 FindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera, int depth)
{
	Intersection intersection = FindIntersection(grid, scene, const_cast<Ray&>(ray), false);
	glm::vec3 finalCol = glm::vec3(0.0f);
	glm::vec3 eyeDir = glm::normalize(-ray.direction);

	if (!intersection.didHit)
	{
		return glm::vec3(0.0f);
	}

	if (intersection.didHit)
	{
		std::vector<DirectionalLight*> dirLights = scene->GetDirLights();
		for (auto& dirLight : dirLights)
		{
			glm::vec3 normalizedLightDirection = glm::normalize(dirLight->direction);
			float nDotL = glm::dot(intersection.hitObjectNormal, normalizedLightDirection);

			if (intersection.hitHasTexture)
			{
				intersection.hitObjectDiffuse = intersection.hitObjectTexture->value(0, 0, intersection.intersectionPoint);
			}
			glm::vec3 lambert = intersection.hitObjectDiffuse * dirLight->colour * std::max(nDotL, 0.0f);

			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection.hitObjectNormal, halfVec);
			glm::vec3 phong = intersection.hitObjectSpecular * dirLight->colour * pow(std::max(nDotH, 0.0f), intersection.hitObjectShininess);
			bounces = 0;

			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection.intersectionPoint + (intersection.hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection shadowIntersection = FindIntersection(grid, scene, shadowRay, true);
			if (shadowIntersection.didHit)
			{
				continue;
			}

			finalCol += lambert + phong;
		}

		std::vector<PointLight*> pointLights = scene->GetPointLights();

		for (auto& pointLight : pointLights)
		{
			glm::vec3 lightDir = pointLight->position - intersection.intersectionPoint;
			float distanceToLight = glm::length(lightDir);
			glm::vec3 normalizedLightDirection = glm::normalize(lightDir);
			float nDotL = glm::dot(intersection.hitObjectNormal, normalizedLightDirection);
			if (intersection.hitHasTexture)
			{
				intersection.hitObjectDiffuse = intersection.hitObjectTexture->value(0, 0, intersection.intersectionPoint);
			}

			float attenuation = 1.0f / (constant + linear * distanceToLight + quadratic * (distanceToLight * distanceToLight));

			glm::vec3 lambert = intersection.hitObjectDiffuse * pointLight->colour * std::max(nDotL, 0.0f);
			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection.hitObjectNormal, halfVec);
			glm::vec3 phong = intersection.hitObjectSpecular * pointLight->colour * pow(std::max(nDotH, 0.0f), intersection.hitObjectShininess);

			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection.intersectionPoint + (intersection.hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection shadowIntersection = FindIntersection(grid, scene, shadowRay, true);
			if (shadowIntersection.didHit)
			{
				float distanceToOccluder = glm::length(shadowIntersection.intersectionPoint - intersection.intersectionPoint);

				if (distanceToOccluder < distanceToLight)
				{
					continue;
				}
			}
			else
			{
			}

			finalCol += (lambert + phong) * attenuation;
		}
		if (depth >= maxDepth)
		{
			return finalCol + intersection.hitObjectEmission + intersection.hitObjectAmbient;
		}

		glm::vec3 reflectDir = glm::reflect(ray.direction, intersection.hitObjectNormal);
		glm::vec3 origin = intersection.intersectionPoint + (intersection.hitObjectNormal * 0.001f);
		Ray mirrorRay(origin, reflectDir);
		finalCol += FindColor(grid, mirrorRay, scene, camera, depth + 1) * intersection.hitObjectSpecular;

		//glm::vec3 refractDir = glm::refract(ray.direction, intersection.hitObjectNormal, 1.0f / intersection.hitObjectIOR);
		//if (glm::length(refractDir) > 0.0f)
		//{
		//	Ray refractRay(origin, glm::normalize(refractDir));
		//	finalCol += FindColor(refractRay, scene, camera, depth + 1) * 0.5f * intersection.hitObjectSpecular;
		//}



		return finalCol + intersection.hitObjectEmission + intersection.hitObjectAmbient;
	}
	else
	{
		return finalCol = glm::vec3(0.0f, 0.0f, 0.0f);
	}
}

glm::vec3 AnalyticFindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera)
{
	Intersection intersection = FindIntersection(grid, scene, const_cast<Ray&>(ray), false);

	if (intersection.didHit)
	{
		QuadLight* light = scene->GetQuadLights()[0];
		light->GetThetaForAllVertices(intersection.intersectionPoint);
		light->GetGammaForAllVertices(intersection.intersectionPoint);
		glm::vec3 phi = light->GetPhi();
		return (intersection.hitObjectDiffuse / (float)M_PI) * (light->intensity) * glm::dot(phi, intersection.hitObjectNormal) + intersection.hitObjectEmission;
	}

	return glm::vec3(0.0, 0.0, 0.0);
}

glm::vec3 MonteCarloFindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera, int depth, int samples, bool stratify, const Intersection& intersection, bool isIndirect)
{

	std::vector<QuadLight*> lights = scene->GetQuadLights();
	glm::vec3 directLight = glm::vec3(0.0f);
	for (QuadLight* light : lights)
	{
		if (intersection.didHit)
		{
			if (!isIndirect)
				return intersection.hitObjectEmission;


			// double-precision intermediate for more robust result
			glm::dvec3 dab = glm::dvec3(light->ab);
			glm::dvec3 dac = glm::dvec3(light->ac);
			double angleABd = glm::clamp(glm::dot(glm::normalize(dab), glm::normalize(dac)), -1.0, 1.0);
			double lightAread = glm::length(glm::cross(dab, dac));
			float lightArea = static_cast<float>(lightAread);
			glm::vec3 perLight = glm::vec3(0.0f);

			if (stratify)
			{
				int M = (int)sqrt(samples);

				for (int i = 0; i < M; i++)
				{
					for (int j = 0; j < M; j++)
					{
						float u1 = RandFloat();
						float u2 = RandFloat();
						glm::vec3 sampleLightPoint = light->a + ((j + u1) / M) * light->ab + ((i + u2) / M) * light->ac;
						glm::vec3 toLight = sampleLightPoint - intersection.intersectionPoint;
						float distanceToLight = glm::length(toLight);
						glm::vec3 dir = toLight / distanceToLight;

						glm::vec3 origin = intersection.intersectionPoint + intersection.hitObjectNormal * 0.001f;
						Ray sampleRay(origin, dir);
						Intersection lightSample = FindIntersection(grid, scene, sampleRay, false);

						const float shadowEpsilon = 1e-2f;
						if (!lightSample.didHit)
						{
							continue;
						}

						if (!lightSample.isLight)
						{
							continue;
						}

						if (std::abs(lightSample.t - distanceToLight) > shadowEpsilon)
						{
							continue;
						}

						glm::vec3 eyeDir = (camera->getEyePos() - intersection.intersectionPoint);
						glm::vec3 normWO = glm::normalize(eyeDir);
						glm::vec3 reflectVector = glm::reflect(normWO, intersection.hitObjectNormal);

						glm::vec3 lightNormal = glm::normalize(glm::cross(-light->ab, -light->ac));

						float reflectVecDotWi = glm::dot(reflectVector, dir);
						glm::vec3 brdf = (intersection.hitObjectDiffuse / (float)M_PI) + (intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2) / (float)(M_PI * 2.0f))) * (glm::pow(reflectVecDotWi, intersection.hitObjectShininess));
						float G = (1.0f / glm::pow(glm::length(sampleLightPoint - intersection.intersectionPoint), 2.0f)) * std::max(glm::dot(intersection.hitObjectNormal, dir), 0.0f) * std::max(glm::dot(lightNormal, dir), 0.0f);
						perLight += brdf * G;
					}
				}
			}
			else
			{

				for (int i = 0; i < samples; i++)
				{
					float u1 = RandFloat();
					float u2 = RandFloat();
					glm::vec3 sampleLightPoint = light->a + u1 * light->ab + u2 * light->ac;
					glm::vec3 dir = glm::normalize(sampleLightPoint - intersection.intersectionPoint);
					glm::vec3 origin = intersection.intersectionPoint + (intersection.hitObjectNormal * 0.01f);
					Ray sampleRay(origin, dir);
					Intersection lightSample = FindIntersection(grid, scene, sampleRay, true);

					if (!lightSample.isLight)
						continue;

					glm::vec3 eyeDir = (camera->getEyePos() - intersection.intersectionPoint);
					glm::vec3 normWO = glm::normalize(eyeDir);
					glm::vec3 reflectVector = glm::reflect(normWO, intersection.hitObjectNormal);

					glm::vec3 lightNormal = glm::normalize(glm::cross(-light->ab, -light->ac));

					float reflectVecDotWi = glm::dot(reflectVector, dir);
					glm::vec3 brdf = (intersection.hitObjectDiffuse / (float)M_PI) + (intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2) / (float)(M_PI * 2.0f))) * (glm::pow(reflectVecDotWi, intersection.hitObjectShininess));
					float G = (1.0f / glm::pow(glm::length(sampleLightPoint - intersection.intersectionPoint), 2.0f)) * std::max(glm::dot(intersection.hitObjectNormal, dir), 0.0f) * std::max(glm::dot(lightNormal, dir), 0.0f);
					perLight += brdf * G;
				}
			}

			directLight += perLight * light->intensity * (lightArea / (float)samples);
		}
	}
	return directLight;
}



glm::vec3 PathTracerFindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera, int depth, int samples, bool stratify, const Intersection& intersection, bool useNEE, bool useRR, glm::vec3 throughput, std::string importanceSampling)
{
	glm::vec3 accumCol(0.0f);
	if (!intersection.didHit)
	{
		return glm::vec3(0.0f);
	}

	//if (intersection.isLight && depth >= 1)
	//{

	//	return glm::vec3(0.0f);
	//}



	//if (depth >= maxDepth)
	//{
	//	return accumCol;
	//	//	return glm::vec3(0.0f);
	//}


	glm::vec3 directLight = glm::vec3(0.0f);
	if (useNEE)
	{
		if (depth == 0)
		{
			accumCol += throughput * intersection.hitObjectEmission;
		}

		std::vector<QuadLight*> lights = scene->GetQuadLights();
		for (QuadLight* light : lights)
		{
			float lightAread = glm::length(glm::cross(light->ab, light->ac));
			float lightArea = static_cast<float>(lightAread);
			glm::vec3 perLight = glm::vec3(0.0f);


			for (int i = 0; i < samples; i++)
			{
				float u1 = RandFloatOne();
				float u2 = RandFloatOne();
				glm::vec3 sampleLightPoint = light->a + u1 * light->ab + u2 * light->ac;
				glm::vec3 toLight = sampleLightPoint - intersection.intersectionPoint;
				float distanceToLight = glm::length(toLight);
				glm::vec3 dir = toLight / distanceToLight;

				glm::vec3 origin = intersection.intersectionPoint + intersection.hitObjectNormal * 0.001f;
				Ray sampleRay(origin, dir);
				Intersection lightSample = FindIntersection(grid, scene, sampleRay, false);

				const float shadowEpsilon = 1e-2f;
				if (!lightSample.didHit)
				{
					continue;
				}

				if (!lightSample.isLight)
				{
					continue;
				}

				if (std::abs(lightSample.t - distanceToLight) > shadowEpsilon)
				{
					continue;
				}
				glm::vec3 lightNormal = glm::normalize(glm::cross(-light->ab, -light->ac));
				glm::vec3 wo = glm::normalize(ray.origin - intersection.intersectionPoint);
				glm::vec3 wi = dir;
				glm::vec3 reflectVector = glm::reflect(-wi, intersection.hitObjectNormal);
				float reflectVecDotWo = std::max(glm::dot(reflectVector, wo), 0.0f);
				glm::vec3 brdf = (intersection.hitObjectDiffuse / (float)M_PI) + (intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2) / (float)(M_PI * 2.0f))) * (glm::pow(reflectVecDotWo, intersection.hitObjectShininess));
				float G = (1.0f / glm::pow(glm::length(sampleLightPoint - intersection.intersectionPoint), 2.0f)) * std::max(glm::dot(intersection.hitObjectNormal, dir), 0.0f) * std::max(glm::dot(lightNormal, dir), 0.0f);
				perLight += brdf * G;
			}

			directLight += perLight * light->intensity * (lightArea / (float)samples);

		}
	}
	//accumCol += directLight;
	accumCol += throughput * directLight;

	float xi_1 = RandFloat();
	float xi_2 = RandFloat();


	float phi = 2.0f * (float)M_PI * xi_2;
	float z = glm::sqrt(xi_1);
	float theta = glm::acos(z);

	glm::vec3 s = glm::vec3(0.0f);
	s.x = glm::cos(phi) * glm::sin(theta);
	s.y = glm::sin(phi) * glm::sin(theta);
	s.z = glm::cos(theta);

	glm::vec3 w = glm::normalize(intersection.hitObjectNormal);
	glm::vec3 helper = (std::abs(w.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
	glm::vec3 u = glm::normalize(glm::cross(helper, w));
	glm::vec3 v = glm::cross(w, u);

	glm::vec3 w_i = s.x * u + s.y * v + s.z * w;

	Ray secondaryRay(intersection.intersectionPoint + intersection.hitObjectNormal * 0.001f, glm::normalize(w_i));
	Intersection secondaryIntersection = FindIntersection(grid, scene, secondaryRay, false);
	glm::vec3 wo = glm::normalize(ray.origin - intersection.intersectionPoint);
	glm::vec3 wi = glm::normalize(w_i);

	glm::vec3 R = glm::reflect(-wi, intersection.hitObjectNormal);
	float spec = std::pow(std::max(glm::dot(R, wo), 0.0f), intersection.hitObjectShininess);


	float pdf = 0.0f;
	float cosTheta = std::max(glm::dot(intersection.hitObjectNormal, wi), 0.0f);
	glm::vec3 brdf;
	float xi0 = RandFloat();
	float xi1 = RandFloat();
	float xi2 = RandFloat();

	if (importanceSampling == "hemisphere")
	{
		brdf =
			(intersection.hitObjectDiffuse / (float)M_PI) +
			(intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2.0f) / (2.0f * (float)M_PI))) * spec;

	}
	else if (importanceSampling == "cosine")
	{

		brdf =
			(intersection.hitObjectDiffuse / (float)M_PI) +
			(intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2.0f) / (2.0f * (float)M_PI)) * spec);
	}
	else if (importanceSampling == "brdf")
	{
		float ks_avg = (intersection.hitObjectSpecular.r + intersection.hitObjectSpecular.g + intersection.hitObjectSpecular.b) / 3.0f;
		float kd_avg = (intersection.hitObjectDiffuse.r + intersection.hitObjectDiffuse.g + intersection.hitObjectDiffuse.b) / 3.0f;

		float t = ks_avg / (ks_avg + kd_avg + 1e-6f); // Avoid division by zero

		float sum = kd_avg + ks_avg;
		float pSpec = (sum > 0.0f) ? (ks_avg / sum) : 0.0f;
		float pDiffuse = 1.0f - pSpec;

		z = RandFloat();
		float gamma = glm::acos(glm::pow(z, 1 / (intersection.hitObjectShininess + 1)));
		float phi_gamma = 2.0f * (float)M_PI * RandFloat();

		glm::vec3 wo = glm::normalize(ray.origin - intersection.intersectionPoint);
		glm::vec3 wi = glm::normalize(w_i);

		glm::vec3 R = glm::reflect(-wo, intersection.hitObjectNormal);


		glm::vec3 s = glm::vec3(0.0f);
		s.x = glm::cos(phi) * glm::sin(theta);
		s.y = glm::sin(phi) * glm::sin(theta);
		s.z = glm::cos(theta);

		glm::vec3 w = glm::normalize(R);
		glm::vec3 helper = (std::abs(w.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
		glm::vec3 u = glm::normalize(glm::cross(helper, w));
		glm::vec3 v = glm::cross(w, u);

		glm::vec3 w_i = s.x * u + s.y * v + s.z * w;
		if (xi0 <= t)
		{
			brdf =
				t * (intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2.0f) / (2.0f * (float)M_PI)) * spec);

			float phi = 2.0f * (float)M_PI * xi2;

			float theta_diffuse = glm::acos(glm::sqrt(xi1));
			float theta_specular = glm::acos(glm::pow(xi1, 1.0 / (intersection.hitObjectShininess + 1)));

			glm::vec3 s = glm::vec3(0.0f);
			s.x = glm::cos(phi) * glm::sin(theta_specular);
			s.y = glm::sin(phi) * glm::sin(theta_specular);
			s.z = glm::cos(theta_specular);

			glm::vec3 w_i = s.x * u + s.y * v + s.z * w;


			glm::vec3 w = glm::normalize(R);
			glm::vec3 helper = (std::abs(w.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
			glm::vec3 u = glm::normalize(glm::cross(helper, w));
			glm::vec3 v = glm::cross(w, u);

			secondaryRay = Ray(intersection.intersectionPoint + intersection.hitObjectNormal * 0.001f, glm::normalize(w_i));
			secondaryIntersection = FindIntersection(grid, scene, secondaryRay, false);
			wo = glm::normalize(ray.origin - intersection.intersectionPoint);
			wi = glm::normalize(w_i);
			R = glm::reflect(-wo, intersection.hitObjectNormal);

			float spec = std::pow(std::max(glm::dot(R, wi), 0.0f), intersection.hitObjectShininess);


			w_i = s.x * u + s.y * v + s.z * w;



			brdf =
				(intersection.hitObjectDiffuse / (float)M_PI) +
				(intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2.0f) / (2.0f * (float)M_PI))) * spec;



		}
		else
		{

			float phi = 2.0f * (float)M_PI * xi2;

			float theta_diffuse = glm::acos(glm::sqrt(xi1));
			float theta_specular = glm::acos(glm::pow(xi1, 1.0 / (intersection.hitObjectShininess + 1)));

			glm::vec3 s = glm::vec3(0.0f);
			s.x = glm::cos(phi) * glm::sin(theta_diffuse);
			s.y = glm::sin(phi) * glm::sin(theta_diffuse);
			s.z = glm::cos(theta_diffuse);

			w_i = s.x * u + s.y * v + s.z * w;

			secondaryRay = Ray(intersection.intersectionPoint + intersection.hitObjectNormal * 0.001f, glm::normalize(w_i));
			secondaryIntersection = FindIntersection(grid, scene, secondaryRay, false);
			wo = glm::normalize(ray.origin - intersection.intersectionPoint);
			wi = glm::normalize(w_i);

			R = glm::reflect(-wo, intersection.hitObjectNormal);
			float spec = std::pow(std::max(glm::dot(R, wi), 0.0f), intersection.hitObjectShininess);

			glm::vec3 w = glm::normalize(R);
			glm::vec3 helper = (std::abs(w.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
			glm::vec3 u = glm::normalize(glm::cross(helper, w));
			glm::vec3 v = glm::cross(w, u);

			glm::vec3 w_i = s.x * u + s.y * v + s.z * w;

			brdf =
				(intersection.hitObjectDiffuse / (float)M_PI) +
				(intersection.hitObjectSpecular * ((intersection.hitObjectShininess + 2.0f) / (2.0f * (float)M_PI))) * spec;

		}

		float cosTheta = std::max(glm::dot(intersection.hitObjectNormal, wi), 0.0f);
		float pdfDiffuse = cosTheta / (float)M_PI;

		glm::vec3 r = glm::reflect(-wo, intersection.hitObjectNormal);
		float rDotWi = std::max(glm::dot(r, wi), 0.0f);
		float pdfSpec = ((intersection.hitObjectShininess + 1.0f) / (2.0f * (float)M_PI)) *
			std::pow(rDotWi, intersection.hitObjectShininess);

		float pdf = pDiffuse * pdfDiffuse + pSpec * pdfSpec;

	}

	if (useRR)
	{
		if (importanceSampling == "hemisphere")
		{
			throughput *= brdf * std::max(glm::dot(intersection.hitObjectNormal, w_i), 0.0f) * (2.0f * (float)M_PI);

		}
		else if (importanceSampling == "cosine")
		{
			throughput *= brdf * (float)M_PI;
		}
		else if (importanceSampling == "brdf")
		{
			float cosTheta = std::max(glm::dot(intersection.hitObjectNormal, wi), 0.0f);

			float pdfDiffuse = cosTheta / (float)M_PI;

			glm::vec3 r = glm::reflect(-wo, intersection.hitObjectNormal);
			float rDotWi = std::max(glm::dot(r, wi), 0.0f);
			float pdfSpec = ((intersection.hitObjectShininess + 1.0f) / (2.0f * (float)M_PI)) *
				std::pow(rDotWi, intersection.hitObjectShininess);

			float kdAvg = (intersection.hitObjectDiffuse.r + intersection.hitObjectDiffuse.g + intersection.hitObjectDiffuse.b) / 3.0f;
			float ksAvg = (intersection.hitObjectSpecular.r + intersection.hitObjectSpecular.g + intersection.hitObjectSpecular.b) / 3.0f;
			float sum = kdAvg + ksAvg;
			float pSpec = (sum > 0.0f) ? (ksAvg / sum) : 0.0f;
			float pDiffuse = 1.0f - pSpec;

			float pdf = pDiffuse * pdfDiffuse + pSpec * pdfSpec;

			if (pdf <= 1e-8f || cosTheta <= 0.0f)
			{
				return accumCol;
			}

			throughput *= brdf * cosTheta / pdf;
		}

		float q = 1.0f - std::min(std::max(throughput.r, std::max(throughput.g, throughput.b)), 1.0f);

		if (q > RandFloatOne())
		{
			return accumCol;
		}
		else
		{
			throughput /= (1.0f - q);
		}
	}

	accumCol += PathTracerFindColor(grid, secondaryRay, scene, camera, depth + 1, samples, stratify, secondaryIntersection, useNEE, useRR, throughput, importanceSampling);
	return accumCol;



}


struct Vertex
{
	glm::vec3 position;
};


std::vector<Vertex> verts;

int RenderPixels(int heightChunkStart, int heightChunk, Scene& scene, Camera& cam, std::string integrator, int width, int height, UniformGrid& grid, int lightSamples, bool lightStratify, BYTE* pixels, int spp, bool useNEE, bool useRR, std::string importanceSampling)
{
	{
		std::ostringstream oss;
		oss << "Thread " << std::this_thread::get_id()
			<< " rows [" << heightChunkStart << ", " << heightChunk << ")";
		LogLine(oss.str());
	}
	for (int y = heightChunkStart; y < heightChunk; y++)
	{
		for (int x = 0; x < width; x++)
		{

			glm::vec3 col(0.0f);
			if (integrator != "pathtracer")
			{

				int depth = 0;
				Ray ray = ShootRay(cam, x, y, width, height);
				Intersection intersection = FindIntersection(&grid, &scene, ray, false);
				if (integrator == "raytracer")
				{
					col = FindColor(&grid, ray, &scene, &cam, depth);
				}
				else if (integrator == "analyticdirect")
				{
					col = AnalyticFindColor(&grid, ray, &scene, &cam);
				}
				else if (integrator == "direct")
				{
					col = MonteCarloFindColor(&grid, ray, &scene, &cam, depth, lightSamples, lightStratify, intersection, useNEE);
				}
			}
			else if (integrator == "pathtracer")
			{
				glm::vec3 accumCol(0.0f);
				for (int pixSample = 0; pixSample < spp; pixSample++)
				{
					float jitterX = 0.0f;
					float jitterY = 0.0f;
					int depth = 0;
					if (pixSample > 0)
					{
						// Jitter the ray within the pixel for anti-aliasing
						jitterX = RandFloat() * 2.0f - 1.0f; // Random value in [-1, 1]
						jitterY = RandFloat() * 2.0f - 1.0f;
						jitterX *= 0.5f; // Scale jitter to half a pixel
						jitterY *= 0.5f;
					}
					else
					{
						jitterX = 0.0f;
						jitterY = 0.0f;
					}
					float pixSampleX = std::max(x + jitterX, 0.0f);
					float pixSampleY = std::max(y + jitterY, 0.0f);
					Ray ray = ShootRay(cam, pixSampleX, pixSampleY, width, height);
					Intersection intersection = FindIntersection(&grid, &scene, ray, false);

					if (!intersection.didHit)
					{
						accumCol += glm::vec3(0.0f);
					}
					else
					{
						glm::vec3 throughput = glm::vec3(1.0f);
						accumCol += PathTracerFindColor(&grid, ray, &scene, &cam, depth, lightSamples, lightStratify, intersection, useNEE, useRR, throughput, importanceSampling);



						//	accumCol += MonteCarloFindColor(&grid, ray, &scene, &cam, depth, 1, lightStratify, intersection);
						//accumCol += intersection.hitObjectEmission;
					}
				}

				accumCol /= static_cast<float>(spp); // Average the samples for anti-aliasing
				col = accumCol;
			}
			int idx = (y * width + x) * 3;
			pixels[idx + 0] = std::min(col.b * 255.0f, 255.0f);
			pixels[idx + 1] = std::min(col.g * 255.0f, 255.0f);
			pixels[idx + 2] = std::min(col.r * 255.0f, 255.0f);
			//	std::cout << "Pixel (" << x << ", " << y << ") of total (" << width << ", " << height << ")" << std::endl;
		}
		int completed = ++g_rowsCompleted;
		if ((completed % 20) == 0 || completed == height)
		{
			std::ostringstream oss;
			oss << "Progress: " << completed << " / " << height << " rows";
			LogLine(oss.str());
		}
	}

	return 1;
}
int main() {
	std::string fname = "outfile.png";
	FreeImage_Initialise();

	glm::vec3 eyePos = glm::vec3(0.0f), center = glm::vec3(0.0f), up = glm::vec3(0.0f);
	int width = 480;
	int height = 480;
	float eyeX, eyeY, eyeZ;
	float centerX, centerY, centerZ;
	float upX, upY, upZ;
	float fovY;
	float ambientR = 0, ambientG = 0, ambientB = 0;
	float diffuseR = 0, diffuseG = 0, diffuseB = 0;
	float specularR = 0, specularG = 0, specularB = 0;
	float shininess = 1.0f;
	float emissionR = 0, emissionG = 0, emissionB = 0;
	float dirLightX, dirLightY, dirLightZ;
	float dirLightR, dirLightG, dirLightB;
	float sphereX, sphereY, sphereZ, sphereRadius;
	int maxVerts = 0;
	int maxVertNorms = 0;
	int lightSamples = 1;
	bool lightStratify = false;
	std::string integrator;
	glm::vec3 a, ab, ac, intensity;
	int spp = 1;
	bool useNEE = false;
	bool useRR = false;
	std::string importanceSampling;

	Scene* scene = new Scene();
	UniformGrid* grid = new UniformGrid();

	std::ifstream file("C:/dev/CSE168x/Release/cornellBRDF.test");
	std::string line;

	while (std::getline(file, line))
	{
		// Remove null characters from the line
		line.erase(std::remove(line.begin(), line.end(), '\0'), line.end());

		if (line.empty())
			continue;

		// Process clean line

		if (line[0] == '#') {
			std::cout << line << std::endl;
			continue;
		}

		std::istringstream iss(line);
		std::string cmd;
		iss >> cmd;

		if (cmd == "popTransform")
		{
			std::cout << line << std::endl;
			modelStack = prevMatrix;

			if (transformStack.size() > 1)
			{
				transformStack.pop_back();
			}
		}

		if (cmd == "pushTransform")
		{
			std::cout << line << std::endl;
			transformStack.push_back(transformStack.back());
		}

		if (cmd == "translate")
		{
			std::cout << line << std::endl;
			float x, y, z;
			iss >> x >> y >> z;
			transformStack.back() = glm::translate(transformStack.back(), glm::vec3(x, y, z));
		}

		if (cmd == "scale")
		{
			std::cout << line << std::endl;
			float x, y, z;
			iss >> x >> y >> z;
			transformStack.back() = glm::scale(transformStack.back(), glm::vec3(x, y, z));
			scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(x, y, z));
		}

		if (cmd == "rotate")
		{
			std::cout << line << std::endl;
			float x, y, z, angle;
			iss >> x >> y >> z >> angle;

			glm::vec3 axis(x, y, z);
			float angleRadians = glm::radians(angle);

			transformStack.back() = glm::rotate(transformStack.back(), angleRadians, axis);
		}

		if (cmd == "size")
		{
			int w, h;
			iss >> w >> h;
			width = w;
			height = h;
		}

		if (cmd == "maxdepth")
		{
			int d;
			iss >> d;
			maxDepth = d;
			if (maxDepth <= 0)
			{
				maxDepth = std::numeric_limits<int>::max();
			}
		}

		if (cmd == "output")
		{
			std::string outputFile;
			iss >> outputFile;
			fname = outputFile;
		}

		if (cmd == "camera") {
			std::cout << line << std::endl;
			iss >> eyeX >> eyeY >> eyeZ >> centerX >> centerY >> centerZ >> upX >> upY >> upZ >> fovY;
		}

		if (cmd == "sphere")
		{
			std::cout << line << std::endl;
			iss >> sphereX >> sphereY >> sphereZ >> sphereRadius;
			scene->AddSphere(new Sphere(glm::vec3(sphereX, sphereY, sphereZ), sphereRadius, glm::vec3(diffuseR, diffuseG, diffuseB), glm::vec3(specularR, specularG, specularB), glm::vec3(emissionR, emissionG, emissionB), shininess, glm::vec3(ambientR, ambientG, ambientB), 1.0f, transformStack.back()));
		}

		if (cmd == "maxverts")
		{
			std::cout << line << std::endl;
			iss >> cmd >> maxVerts;
		}

		if (cmd == "maxvertnorms")
		{
			std::cout << line << std::endl;
			iss >> cmd >> maxVertNorms;
		}

		if (cmd == "vertex")
		{
			std::cout << line << std::endl;
			float vertX, vertY, vertZ;
			iss >> vertX >> vertY >> vertZ;
			verts.push_back(Vertex{ glm::vec3(vertX, vertY, vertZ) });
		}

		if (cmd == "vertexnormal")
		{
			std::cout << line << std::endl;
		}

		if (cmd == "tri")
		{
			std::cout << line << std::endl;
			int v0, v1, v2;
			iss >> v0 >> v1 >> v2;

			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(verts[v0].position, 1.0f), transformStack.back() * glm::vec4(verts[v1].position, 1.0f), transformStack.back() * glm::vec4(verts[v2].position, 1.0f), glm::vec3(diffuseR, diffuseG, diffuseB), glm::vec3(specularR, specularG, specularB), glm::vec3(emissionR, emissionG, emissionB), shininess, glm::vec3(ambientR, ambientG, ambientB), false, NULL));
		}

		if (cmd == "trinormal")
		{
			std::cout << line << std::endl;
		}

		if (cmd == "directional")
		{
			std::cout << line << std::endl;
			iss >> dirLightX >> dirLightY >> dirLightZ >> dirLightR >> dirLightG >> dirLightB;
			scene->AddDirectionalLight(new DirectionalLight(glm::vec3(dirLightX, dirLightY, dirLightZ), glm::vec3(dirLightR, dirLightG, dirLightB)));
		}

		if (cmd == "point")
		{
			std::cout << line << std::endl;
			float x, y, z, r, g, b;
			iss >> x >> y >> z >> r >> g >> b;
			scene->AddPointLight(new PointLight(glm::vec3(x, y, z), glm::vec3(r, g, b)));
		}

		if (cmd == "attenuation")
		{
			std::cout << line << std::endl;
			iss >> constant >> linear >> quadratic;
		}

		if (cmd == "ambient")
		{
			iss >> ambientR >> ambientG >> ambientB;
		}

		if (cmd == "diffuse")
		{
			std::cout << line << std::endl;
			iss >> diffuseR >> diffuseG >> diffuseB;
		}

		if (cmd == "specular")
		{
			std::cout << line << std::endl;
			iss >> specularR >> specularG >> specularB;
		}

		if (cmd == "shininess")
		{
			std::cout << line << std::endl;
			iss >> shininess;
		}

		if (cmd == "emission")
		{
			std::cout << line << std::endl;
			iss >> emissionR >> emissionG >> emissionB;
		}

		if (cmd == "integrator")
		{
			std::cout << line << std::endl;
			iss >> integrator;
		}

		if (cmd == "quadLight")
		{
			std::cout << line << std::endl;
			iss >> a.x >> a.y >> a.z >> ab.x >> ab.y >> ab.z >> ac.x >> ac.y >> ac.z >> intensity.r >> intensity.g >> intensity.b;
			glm::vec3 v0 = a;
			glm::vec3 v1 = a + ab;
			glm::vec3 v2 = a + ab + ac;
			glm::vec3 v3 = a + ac;
			QuadLight* quadLight = new QuadLight(a, ab, ac, intensity);
			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(v0, 1.0f), transformStack.back() * glm::vec4(v1, 1.0f), transformStack.back() * glm::vec4(v2, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), intensity, shininess, glm::vec3(0.0f), true, NULL));
			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(v0, 1.0f), transformStack.back() * glm::vec4(v2, 1.0f), transformStack.back() * glm::vec4(v3, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), intensity, shininess, glm::vec3(0.0f), true, NULL));
			scene->AddQuadLight(quadLight);
		}

		if (cmd == "lightsamples")
		{
			std::cout << line << std::endl;
			iss >> lightSamples;
		}

		if (cmd == "lightstratify")
		{
			std::cout << line << std::endl;
			std::string stratify;
			iss >> stratify;
			if (stratify == "on")
			{
				lightStratify = true;
			}
			else
			{
				lightStratify = false;
			}
		}

		if (cmd == "spp")
		{
			std::cout << line << std::endl;
			iss >> spp;
		}

		if (cmd == "nexteventestimation")
		{
			std::cout << line << std::endl;
			std::string shouldUseNEE;
			iss >> shouldUseNEE;
			useNEE = shouldUseNEE == "on" ? true : false;
		}

		if (cmd == "russianroulette")
		{
			std::cout << line << std::endl;
			std::string shouldUseRR;
			iss >> shouldUseRR;
			useRR = shouldUseRR == "on" ? true : false;
		}

		if (cmd == "importancesampling")
		{
			std::cout << line << std::endl;
			iss >> importanceSampling;
		}
	}

	Camera cam(glm::vec3(eyeX, eyeY, eyeZ), glm::vec3(centerX, centerY, centerZ), glm::vec3(upX, upY, upZ), glm::radians(fovY));

	const int IMAGE_WIDTH = width;
	const int IMAGE_HEIGHT = height;

	BuildUniformGrid(*grid, scene);

	BYTE* pixels = (BYTE*)malloc(sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);
	if (!pixels) {
		FreeImage_DeInitialise();
		return 1;
	}
	memset(pixels, 0, sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);

	glm::vec3 col = glm::vec3(0.0f, 0.0f, 0.0f);


	unsigned int numThreads = std::thread::hardware_concurrency();
	if (numThreads == 0) numThreads = 1;
	// don't spawn more threads than rows (optional)
	if (numThreads > static_cast<unsigned int>(IMAGE_HEIGHT)) numThreads = static_cast<unsigned int>(IMAGE_HEIGHT);

	int baseChunk = IMAGE_HEIGHT / static_cast<int>(numThreads);
	int remainder = IMAGE_HEIGHT % static_cast<int>(numThreads);

	std::vector<std::thread> threads;
	threads.reserve(numThreads);

	for (unsigned int i = 0; i < numThreads; ++i)
	{
		int start = static_cast<int>(i) * baseChunk + std::min(static_cast<int>(i), remainder);
		int extra = (static_cast<int>(i) < remainder) ? 1 : 0;
		int end = start + baseChunk + extra;

		if (start >= end) continue; // nothing to do for this thread

		threads.emplace_back(RenderPixels, start, end, std::ref(*scene), std::ref(cam), integrator, IMAGE_WIDTH, IMAGE_HEIGHT, std::ref(*grid), lightSamples, lightStratify, pixels, spp, useNEE, useRR, importanceSampling);
	}

	for (auto& t : threads)
	{
		if (t.joinable())
			t.join();
	}
	//	RenderPixels(0, IMAGE_HEIGHT, *scene, cam, integrator, IMAGE_WIDTH, IMAGE_HEIGHT, *grid, lightSamples, lightStratify, pixels, 1);

	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}

