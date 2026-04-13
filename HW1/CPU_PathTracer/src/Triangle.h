#pragma once
#include "glm/glm.hpp"

class Triangle
{
public:
	Triangle(glm::vec3 vertex0, glm::vec3 vertex1, glm::vec3 vertex2, glm::vec3 diffuse, glm::vec3 specular, glm::vec3 emission, float shininess, glm::vec3 ambient)
		:
		vertex0(vertex0),
		vertex1(vertex1),
		vertex2(vertex2),
		diffuse(diffuse),
		specular(specular),
		emission(emission),
		shininess(shininess),
		ambient(ambient)

	{
		normalA = normalize(cross((vertex2 - vertex0), (vertex1 - vertex0)));
		normalB = normalize(cross((vertex0 - vertex1), (vertex2 - vertex1)));
		normalC = normalize(cross((vertex1 - vertex2), (vertex0 - vertex2)));
	}

	Triangle()
	{
	}

	void SetNormal(glm::vec3 newNormal)
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

};