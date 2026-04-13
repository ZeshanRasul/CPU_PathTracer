#pragma once
#include <vector>

#include "Sphere.h"

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

private:
	std::vector<Sphere*> spheres;
};