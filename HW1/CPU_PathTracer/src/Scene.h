#pragma once
#include <vector>

#include "Sphere.h"
#include "Triangle.h"
#include "DirectionalLight.h"

class Scene {
public:
	void AddSphere(Sphere* sphere)
	{
		spheres.push_back(sphere);
	}

	std::vector<Sphere*> GetSpheres()
	{
		return spheres;
	}

	void AddTriangle(Triangle* tri)
	{
		triangles.push_back(tri);
	}

	std::vector<Triangle*> GetTriangles()
	{
		return triangles;
	}

	void AddDirectionalLight(DirectionalLight* dirLight)
	{
		dirLights.push_back(dirLight);
	}

	std::vector<DirectionalLight*> GetDirLights()
	{
		return dirLights;
	}

private:
	std::vector<Sphere*> spheres;
	std::vector<Triangle*> triangles;
	std::vector<DirectionalLight*> dirLights;
};