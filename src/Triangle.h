#pragma once
#include "../include/glm/glm.hpp"
#include "Texture.h"

class Triangle
{
public:
	Triangle(glm::vec3 vertex0, glm::vec3 vertex1, glm::vec3 vertex2, glm::vec3 diffuse, glm::vec3 specular, glm::vec3 emission, float shininess, glm::vec3 ambient, std::string brdf, float roughness, bool isAreaLight = false, texture* matTex = NULL)
		:
		vertex0(vertex0),
		vertex1(vertex1),
		vertex2(vertex2),
		diffuse(diffuse),
		specular(specular),
		emission(emission),
		shininess(shininess),
		ambient(ambient),
		brdf(brdf),
		roughness(roughness),
		matTexture(matTex),
		isLight(isAreaLight)

	{
		normalA = glm::normalize(glm::cross((vertex2 - vertex0), (vertex1 - vertex0)));
		normalB = glm::normalize(glm::cross((vertex0 - vertex1), (vertex2 - vertex1)));
		normalC = glm::normalize(glm::cross((vertex1 - vertex2), (vertex0 - vertex2)));

		glm::vec3 e1 = vertex1 - vertex0;
		glm::vec3 e2 = vertex2 - vertex0;

		glm::vec3 faceNormal = glm::normalize(glm::cross(e1, e2));

		SetNormal(faceNormal);

	}

	Triangle()
	{
	}

	void SetNormal(const glm::vec3& newNormal)
	{
		normal = newNormal;
	}


	glm::vec3 vertex0, vertex1, vertex2;
	glm::vec3 normalA;
	glm::vec3 normalB;
	glm::vec3 normalC;
	glm::vec3 normal;
	glm::vec3 diffuse, specular, emission, ambient;
	float shininess;
	texture* matTexture;
	bool isLight = false;
	std::string brdf = "phong";
	float roughness;
};