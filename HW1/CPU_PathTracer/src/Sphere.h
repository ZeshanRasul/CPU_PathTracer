#pragma once
#include "glm/glm.hpp"
#include "Texture.h"

class Sphere
{
public:
	Sphere(glm::vec3 center, glm::vec3 radius, glm::vec3 diffuse, glm::vec3 specular, glm::vec3 emission, float shininess, glm::vec3 ambient, float ior, texture* matTex = NULL)
		:
		center(center),
		radius(radius),
		diffuse(diffuse),
		specular(specular),
		emission(emission),
		shininess(shininess),
		ambient(ambient),
		ior(ior),
		matTexture(matTex)
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
	glm::vec3 radius;
	glm::vec3 normal;
	glm::vec3 diffuse, specular, emission, ambient;
	float shininess;
	float ior;
	texture* matTexture;
};