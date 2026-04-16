#include "Freeimage.h"
#include <string>
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/constants.hpp"

#include "Camera.h"
#include "Ray.h"
#include "Intersection.h"
#include "Scene.h"


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
};

struct GridCell
{
	std::vector<int> sphereIndices;
	std::vector<int> triangleIndices;
};

struct UniformGrid
{
	AABB bounds;
	int nx = 5;
	int ny = 5;
	int nz = 5;
	glm::vec3 cellSize = glm::vec3(1.0f);
	std::vector<GridCell> cells;

	int Index(int x, int y, int z) const
	{
		return x + nx * (y + ny * z);
	}

	bool IsValidCell(int x, int y, int z) const
	{
		return x >= 0 && x < nx &&
			y >= 0 && y < ny &&
			z >= 0 && z < nz;
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
	glm::vec3 r(s->radius);
	box.min = glm::vec3(s->transform * glm::vec4(s->center, 1.0f)) - r;
	box.max = glm::vec3(s->transform * glm::vec4(s->center, 1.0f)) + r;
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

glm::ivec3 WorldToCell(const UniformGrid& grid, const glm::vec3& p)
{
	glm::vec3 rel = (p - grid.bounds.min) / grid.cellSize;

	glm::ivec3 cell(
		static_cast<int>(std::floor(rel.x)),
		static_cast<int>(std::floor(rel.y)),
		static_cast<int>(std::floor(rel.z))
	);

	cell.x = std::clamp(cell.x, 0, grid.nx - 1);
	cell.y = std::clamp(cell.y, 0, grid.ny - 1);
	cell.z = std::clamp(cell.z, 0, grid.nz - 1);

	return cell;
}

void BuildUniformGrid(UniformGrid& grid, Scene* scene)
{
	std::vector<Sphere*>& spheres = scene->GetSpheresRef();
	std::vector<Triangle*> triangles = scene->GetTriangles();

	grid.bounds = ComputeSceneBounds(scene);

	glm::vec3 extent = grid.bounds.max - grid.bounds.min;
	const float minExtent = 3;
	for (int axis = 0; axis < 3; ++axis)
	{
		if (extent[axis] < minExtent)
		{
			float expand = (minExtent - extent[axis]) * 0.5f;
			grid.bounds.min[axis] -= expand;
			grid.bounds.max[axis] += expand;
		}
	}


	grid.cellSize = (grid.bounds.max - grid.bounds.min) /
		glm::vec3((float)grid.nx, (float)grid.ny, (float)grid.nz);

	grid.cells.clear();
	grid.cells.resize(grid.nx * grid.ny * grid.nz);

	// Insert spheres
	for (int i = 0; i < (int)spheres.size(); ++i)
	{
		AABB sphereBox = ComputeSphereAABB(spheres[i]);

		glm::ivec3 minCell = WorldToCell(grid, sphereBox.min);
		glm::ivec3 maxCell = WorldToCell(grid, sphereBox.max);

		for (int z = minCell.z; z <= maxCell.z; ++z)
		{
			for (int y = minCell.y; y <= maxCell.y; ++y)
			{
				for (int x = minCell.x; x <= maxCell.x; ++x)
				{
					grid.cells[grid.Index(x, y, z)].sphereIndices.push_back(i);
				}
			}
		}
	}

	// Insert triangles
	for (int i = 0; i < (int)triangles.size(); ++i)
	{
		AABB triBox = ComputeTriangleAABB(triangles[i]);

		glm::ivec3 minCell = WorldToCell(grid, triBox.min);
		glm::ivec3 maxCell = WorldToCell(grid, triBox.max);

		for (int z = minCell.z; z <= maxCell.z; ++z)
		{
			for (int y = minCell.y; y <= maxCell.y; ++y)
			{
				for (int x = minCell.x; x <= maxCell.x; ++x)
				{
					grid.cells[grid.Index(x, y, z)].triangleIndices.push_back(i);
				}
			}
		}
	}
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
			// Ray parallel to slab: must already be inside
			if (o < bmin || o > bmax)
				return false;
			continue;
		}

		float invD = 1.0f / d;
		float t0 = (bmin - o) * invD;
		float t1 = (bmax - o) * invD;

		if (t0 > t1)
			std::swap(t0, t1);

		tMin = std::max(tMin, t0);
		tMax = std::min(tMax, t1);

		if (tMax < tMin)
			return false;
	}

	tEnter = tMin;
	tExit = tMax;
	return true;
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

	glm::vec3 faceNormal = glm::normalize(glm::cross(e1, e2));

	if (glm::dot(faceNormal, ray.direction) > 0.0f)
	{
		faceNormal = -faceNormal;
	}
	triangle->SetNormal(faceNormal);

	return glm::vec3(t, beta, gamma);
}

bool TraverseUniformGrid(
	UniformGrid& grid,
	Scene* scene,
	Ray& ray,
	Intersection& closestHit)
{
	float tEnter, tExit;
	if (!IntersectAABB(ray, grid.bounds, tEnter, tExit))
		return false;

	std::vector<Sphere*>& spheres = scene->GetSpheresRef();
	std::vector<Triangle*> triangles = scene->GetTriangles();

	glm::vec3 startPoint = ray.origin + tEnter * ray.direction;
	glm::ivec3 cell = WorldToCell(grid, startPoint);

	glm::ivec3 step(0);
	glm::vec3 tMax(0.0f);
	glm::vec3 tDelta(0.0f);

	for (int axis = 0; axis < 3; ++axis)
	{
		float d = ray.direction[axis];

		if (d > 0.0f)
		{
			step[axis] = 1;
			float nextBoundary = grid.bounds.min[axis] + (cell[axis] + 1) * grid.cellSize[axis];
			tMax[axis] = tEnter + (nextBoundary - startPoint[axis]) / d;
			tDelta[axis] = grid.cellSize[axis] / d;
		}
		else if (d < 0.0f)
		{
			step[axis] = -1;
			float nextBoundary = grid.bounds.min[axis] + cell[axis] * grid.cellSize[axis];
			tMax[axis] = tEnter + (nextBoundary - startPoint[axis]) / d;
			tDelta[axis] = -grid.cellSize[axis] / d;
		}
		else
		{
			step[axis] = 0;
			tMax[axis] = std::numeric_limits<float>::infinity();
			tDelta[axis] = std::numeric_limits<float>::infinity();
		}
	}

	bool hitAnything = false;
	float bestT = tExit;

	while (grid.IsValidCell(cell.x, cell.y, cell.z))
	{
		const GridCell& currentCell = grid.cells[grid.Index(cell.x, cell.y, cell.z)];

		// Test spheres
		for (int sphereIndex : currentCell.sphereIndices)
		{
			glm::mat4 invTransform = glm::inverse(spheres[sphereIndex]->transform);

			Ray transformedRay(
				glm::vec3(invTransform * glm::vec4(ray.origin, 1.0f)),
				glm::vec3(invTransform * glm::vec4(ray.direction, 0.0f)));

			float sphereTLocal = CheckSphereIntersection(spheres[sphereIndex], transformedRay);

			if (sphereTLocal < INFINITY)
			{
				glm::vec3 localHitPoint = transformedRay.origin + transformedRay.direction * sphereTLocal;
				glm::vec3 worldHitPoint = glm::vec3(spheres[sphereIndex]->transform * glm::vec4(localHitPoint, 1.0f));
				float worldT = glm::length(worldHitPoint - ray.origin);

				if (worldT < bestT)
				{
					bestT = worldT;
					closestHit.t = worldT;
					closestHit.sphereIndex = sphereIndex;
					closestHit.triangleIndex = -1;
					closestHit.hitObjectIsSphere = true;
					hitAnything = true;
				}
			}
		}

		// Test triangles
		for (int triangleIndex : currentCell.triangleIndices)
		{
			glm::vec3 tBetaGamma = CheckTriangleIntersection(triangles[triangleIndex], ray);
			float triT = tBetaGamma.x;

			if (triT > 0.0f && triT < bestT && triT < INFINITY)
			{
				bestT = triT;
				closestHit.t = triT;
				closestHit.sphereIndex = -1;
				closestHit.triangleIndex = triangleIndex;
				closestHit.hitObjectIsSphere = false;
				hitAnything = true;
			}
		}

		float nextCrossing = std::min(tMax.x, std::min(tMax.y, tMax.z));

		if (hitAnything && bestT < nextCrossing)
			break;

		if (tMax.x < tMax.y)
		{
			if (tMax.x < tMax.z)
			{
				cell.x += step.x;
				tMax.x += tDelta.x;
			}
			else
			{
				cell.z += step.z;
				tMax.z += tDelta.z;
			}
		}
		else
		{
			if (tMax.y < tMax.z)
			{
				cell.y += step.y;
				tMax.y += tDelta.y;
			}
			else
			{
				cell.z += step.z;
				tMax.z += tDelta.z;
			}
		}
	}

	return hitAnything;
}

Ray ShootRay(const Camera& cam, const int i, const int j, const int width, const int height)
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

Intersection* FindIntersection(UniformGrid* grid, Scene* scene, Ray& ray)
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
	float hitObjectShininess = 0.0f;
	glm::vec3 tBetaGamma2;
	glm::vec3 intersectionPoint(INFINITY, INFINITY, INFINITY);
	bool hitObjectIsSphere = false;
	float hitObjectIOR = 1.0f;
	bool hitHasTexture = false;
	texture* hitObjectTexture = NULL;
	bool isAreaLight = false;
	std::vector<Sphere*> sceneSpheres = scene->GetSpheres();


	Intersection* intersection = new Intersection();
	didHit = TraverseUniformGrid(*grid, scene, ray, *intersection);

	if (didHit)
	{
		if (intersection->hitObjectIsSphere)
		{
			Sphere* hitSphere = sceneSpheres[intersection->sphereIndex];

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

			return new Intersection(
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
				intersection->t,
				false,
				hitSphere->matTexture != NULL,
				hitSphere->matTexture
			);
		}
		else if (intersection->triangleIndex != -1)
		{
			std::vector<Triangle*> sceneTriangles = scene->GetTriangles();
			Triangle* hitTri = sceneTriangles[intersection->triangleIndex];

			glm::vec3 tBetaGamma = CheckTriangleIntersection(hitTri, ray);
			float tTriangle = tBetaGamma.x;
			glm::vec3 intersectionPoint = ray.origin + ray.direction * tTriangle;

			return new Intersection(
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
		}
	}

	return new Intersection(
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
	Intersection* intersection = FindIntersection(grid, scene, const_cast<Ray&>(ray));
	glm::vec3 finalCol = glm::vec3(0.0f);
	glm::vec3 eyeDir = glm::normalize(-ray.direction);

	if (!intersection->didHit)
	{
		delete intersection;
		return glm::vec3(0.0f);
	}

	if (intersection->didHit)
	{
		std::vector<DirectionalLight*> dirLights = scene->GetDirLights();
		for (auto& dirLight : dirLights)
		{
			glm::vec3 normalizedLightDirection = glm::normalize(dirLight->direction);
			float nDotL = glm::dot(intersection->hitObjectNormal, normalizedLightDirection);

			if (intersection->hitHasTexture)
			{
				intersection->hitObjectDiffuse = intersection->hitObjectTexture->value(0, 0, intersection->intersectionPoint);
			}
			glm::vec3 lambert = intersection->hitObjectDiffuse * dirLight->colour * std::max(nDotL, 0.0f);

			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection->hitObjectNormal, halfVec);
			glm::vec3 phong = intersection->hitObjectSpecular * dirLight->colour * pow(std::max(nDotH, 0.0f), intersection->hitObjectShininess);
			bounces = 0;

			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(grid, scene, shadowRay);
			if (shadowIntersection->didHit)
			{
				//finalCol += intersection->hitObjectAmbient;
				delete shadowIntersection;
				continue;
			}

			finalCol += lambert + phong;
		}

		std::vector<PointLight*> pointLights = scene->GetPointLights();

		for (auto& pointLight : pointLights)
		{
			glm::vec3 lightDir = pointLight->position - intersection->intersectionPoint;
			float distanceToLight = glm::length(lightDir);
			glm::vec3 normalizedLightDirection = glm::normalize(lightDir);
			float nDotL = glm::dot(intersection->hitObjectNormal, normalizedLightDirection);
			if (intersection->hitHasTexture)
			{
				intersection->hitObjectDiffuse = intersection->hitObjectTexture->value(0, 0, intersection->intersectionPoint);
			}

			float attenuation = 1.0f / (constant + linear * distanceToLight + quadratic * (distanceToLight * distanceToLight));

			glm::vec3 lambert = intersection->hitObjectDiffuse * pointLight->colour * std::max(nDotL, 0.0f);
			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection->hitObjectNormal, halfVec);
			glm::vec3 phong = intersection->hitObjectSpecular * pointLight->colour * pow(std::max(nDotH, 0.0f), intersection->hitObjectShininess);

			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(grid, scene, shadowRay);
			if (shadowIntersection->didHit)
			{
				float distanceToOccluder = glm::length(shadowIntersection->intersectionPoint - intersection->intersectionPoint);
				delete shadowIntersection;

				if (distanceToOccluder < distanceToLight)
				{
					continue;
				}
			}
			else
			{
				delete shadowIntersection;
			}

			finalCol += (lambert + phong) * attenuation;
		}
		if (depth >= maxDepth)
		{
			return finalCol + intersection->hitObjectEmission + intersection->hitObjectAmbient;
		}

		glm::vec3 reflectDir = glm::reflect(ray.direction, intersection->hitObjectNormal);
		glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
		Ray mirrorRay(origin, reflectDir);
		finalCol += FindColor(grid, mirrorRay, scene, camera, depth + 1) * intersection->hitObjectSpecular;

		//glm::vec3 refractDir = glm::refract(ray.direction, intersection->hitObjectNormal, 1.0f / intersection->hitObjectIOR);
		//if (glm::length(refractDir) > 0.0f)
		//{
		//	Ray refractRay(origin, glm::normalize(refractDir));
		//	finalCol += FindColor(refractRay, scene, camera, depth + 1) * 0.5f * intersection->hitObjectSpecular;
		//}



		return finalCol + intersection->hitObjectEmission + intersection->hitObjectAmbient;
	}
	else
	{
		return finalCol = glm::vec3(0.0f, 0.0f, 0.0f);
	}
}

glm::vec3 AnalyticFindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera)
{
	Intersection* intersection = FindIntersection(grid, scene, const_cast<Ray&>(ray));

	if (intersection->didHit)
	{
		QuadLight* light = scene->GetQuadLights()[0];
		light->GetThetaForAllVertices(intersection->intersectionPoint);
		light->GetGammaForAllVertices(intersection->intersectionPoint);
		glm::vec3 phi = light->GetPhi();
		return (intersection->hitObjectDiffuse / (float)M_PI) * (light->intensity) * glm::dot(phi, intersection->hitObjectNormal) + intersection->hitObjectEmission;
	}

	return glm::vec3(0.0, 0.0, 0.0);
}

glm::vec3 MonteCarloFindColor(UniformGrid* grid, const Ray& ray, Scene* scene, Camera* camera, int depth, int samples)
{
	Intersection* intersection = FindIntersection(grid, scene, const_cast<Ray&>(ray));



	if (intersection->didHit)
	{
		QuadLight* light = scene->GetQuadLights()[0];
		if (intersection->isLight)
			return intersection->hitObjectEmission;

		glm::vec3 directLight = glm::vec3(0.0f);
		float lightArea = -glm::dot((light->v0 - light->v1), (light->v2 - light->v3));
		for (int i = 0; i < samples; i++)
		{
			float u1 = static_cast <float>(rand()) / static_cast <float>(RAND_MAX);
			float u2 = static_cast <float>(rand()) / static_cast <float>(RAND_MAX);
			glm::vec3 sampleLightPoint = light->a + u1 * light->ab + u2 * light->ac;
			glm::vec3 dir = glm::normalize(sampleLightPoint - intersection->intersectionPoint);
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(grid, scene, shadowRay);

			if (shadowIntersection->didHit && !shadowIntersection->isLight)
				continue;

			glm::vec3 eyeDir = (camera->getEyePos() - intersection->intersectionPoint);
			glm::vec3 normWO = glm::normalize(eyeDir);
			glm::vec3 reflectVector = glm::reflect(normWO, intersection->hitObjectNormal);

			glm::vec3 lightNormal = glm::cross(light->ab, light->ac);

			float reflectVecDotWi = glm::dot(reflectVector, dir);
			glm::vec3 brdf = (intersection->hitObjectDiffuse / (float)M_PI) + (intersection->hitObjectSpecular * ((intersection->hitObjectShininess * 2) / (float)M_2_PI)) * (glm::pow(reflectVecDotWi, intersection->hitObjectShininess));
			float G = (1.0f / glm::length(intersection->intersectionPoint - sampleLightPoint)) * std::max(glm::dot(intersection->hitObjectNormal, (glm::normalize(sampleLightPoint - intersection->intersectionPoint))), 0.0f) * glm::dot(lightNormal, (glm::normalize(sampleLightPoint - intersection->intersectionPoint)));
			directLight += brdf * G;
		}

		return directLight * light->intensity * (lightArea / (float)samples);
	}

	return glm::vec3(0.0f, 0.0f, 0.0f);
}


struct Vertex
{
	glm::vec3 position;
};


std::vector<Vertex> verts;

int main() {
	std::string fname = "outfile.png";
	FreeImage_Initialise();

	glm::vec3 eyePos = glm::vec3(0.0f), center = glm::vec3(0.0f), up = glm::vec3(0.0f);
	int width;
	int height;
	float eyeX, eyeY, eyeZ;
	float centerX, centerY, centerZ;
	float upX, upY, upZ;
	float fovY;
	float ambientR = 0, ambientG = 0, ambientB = 0;
	float diffuseR = 0, diffuseG = 0, diffuseB = 0;
	float specularR = 0, specularG = 0, specularB = 0;
	float shininess = 32;
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

	Scene* scene = new Scene();
	UniformGrid* grid = new UniformGrid();

	std::ifstream file("C:/dev/CSE168x/HW1/CPU_PathTracer/Release/direct9.test");
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

			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(verts[v0].position, 1.0f), transformStack.back() * glm::vec4(verts[v1].position, 1.0f), transformStack.back() * glm::vec4(verts[v2].position, 1.0f), glm::vec3(diffuseR, diffuseG, diffuseB), glm::vec3(specularR, specularG, specularB), glm::vec3(emissionR, emissionG, emissionB), shininess, glm::vec3(ambientR, ambientG, ambientB), NULL));
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
			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(v0, 1.0f), transformStack.back() * glm::vec4(v1, 1.0f), transformStack.back() * glm::vec4(v2, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(specularR, specularG, specularB), intensity, shininess, glm::vec3(ambientR, ambientG, ambientB), true, NULL));
			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(v0, 1.0f), transformStack.back() * glm::vec4(v2, 1.0f), transformStack.back() * glm::vec4(v3, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(specularR, specularG, specularB), intensity, shininess, glm::vec3(ambientR, ambientG, ambientB), true, NULL));
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

	for (int y = IMAGE_HEIGHT; y > 0; y--)
	{
		for (int x = 0; x < IMAGE_WIDTH; x++)
		{
			int depth = 0;
			Ray ray = ShootRay(cam, x, y, IMAGE_WIDTH, IMAGE_HEIGHT);
			Intersection* intersection = FindIntersection(grid, scene, ray);
			if (integrator == "raytracer")
			{
				col = FindColor(grid, ray, scene, &cam, depth);
			}
			else if (integrator == "analyticdirect")
			{
				col = AnalyticFindColor(grid, ray, scene, &cam);
			}
			else if (integrator == "direct")
			{
				col = MonteCarloFindColor(grid, ray, scene, &cam, depth, lightSamples);
			}
			int idx = (y * IMAGE_WIDTH + x) * 3;
			pixels[idx + 0] = std::min(col.b * 255.0f, 255.0f);
			// Fix for C6386 and C4244:
			// - Ensure idx is always within bounds: idx + 0, idx + 1, idx + 2 < IMAGE_WIDTH * IMAGE_HEIGHT * 3
			// - Explicitly cast to BYTE to avoid C4244 warning

			if (idx + 2 < IMAGE_WIDTH * IMAGE_HEIGHT * 3) {
				pixels[idx + 0] = static_cast<BYTE>(std::min(std::max(col.b * 255.0f, 0.0f), 255.0f));
				pixels[idx + 1] = static_cast<BYTE>(std::min(std::max(col.g * 255.0f, 0.0f), 255.0f));
				pixels[idx + 2] = static_cast<BYTE>(std::min(std::max(col.r * 255.0f, 0.0f), 255.0f));
			}
			pixels[idx + 1] = std::min(col.g * 255.0f, 255.0f);
			pixels[idx + 2] = std::min(col.r * 255.0f, 255.0f);
			//std::cout << "Pixel (" << x << ", " << y << ") of total (" << IMAGE_WIDTH << ", " << IMAGE_HEIGHT << ")" << std::endl;
		}

	}
	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}