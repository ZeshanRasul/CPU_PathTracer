#pragma once
#include "glm/glm.hpp"

class Sphere
{
public:
	Sphere(glm::vec3 center, float radius, glm::vec3 diffuse, glm::vec3 specular, glm::vec3 emission, float shininess, glm::vec3 ambient, float ior)
		:
		center(center),
		radius(radius),
		diffuse(diffuse),
		specular(specular),
		emission(emission),
		shininess(shininess),
		ambient(ambient),
		ior(ior)
	{
	}

	Sphere()
	{
	}

	~Sphere()
	{
	};

	void SetNormal(glm::vec3 newNormal)
	{
		normal = newNormal;
	}

	glm::vec3 center;
	float radius;
	glm::vec3 normal;
	glm::vec3 diffuse, specular, emission, ambient;
	float shininess;
	float ior;
};