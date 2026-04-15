#pragma once
#include "glm/glm.hpp"
#include "Texture.h"

class Intersection
{
public:
	Intersection(bool didHit,
		glm::vec3 intersectionPoint,
		glm::vec3 hitObjectDiffuse,
		glm::vec3 hitObjectSpecular,
		glm::vec3 hitObjectEmission,
		float hitObjectShininess,
		glm::vec3 hitObjectAmbient,
		glm::vec3 hitObjectNormal,
		glm::vec3 center,
		float hitObjectIOR,
		float tHit,
		bool hitHasTexture = false,
		texture* hitObjectTexture = nullptr
		)
		:
		didHit(didHit),
		intersectionPoint(intersectionPoint),
		hitObjectDiffuse(hitObjectDiffuse),
		hitObjectSpecular(hitObjectSpecular),
		hitObjectEmission(hitObjectEmission),
		hitObjectShininess(hitObjectShininess),
		hitObjectAmbient(hitObjectAmbient),
		hitObjectNormal(hitObjectNormal),
		center(center),
		hitObjectIOR(hitObjectIOR),
		t(tHit),
		hitHasTexture(hitHasTexture),
		hitObjectTexture(hitObjectTexture)
	{
	}

	Intersection()
	{
	}

	~Intersection()
	{
	}

public:
	bool didHit = false;
	glm::vec3 intersectionPoint;
	glm::vec3 hitObjectDiffuse;
	glm::vec3 hitObjectSpecular;
	glm::vec3 hitObjectEmission;
	glm::vec3 hitObjectAmbient;
	float hitObjectShininess;
	glm::vec3 hitObjectNormal;
	bool hitObjectIsSphere;
	glm::vec3 center;
	glm::vec3 reflectionNormal;
	float hitObjectIOR = 1.0f;
	float u;
	float v;
	bool hitHasTexture = false;
	texture* hitObjectTexture = NULL;
	float t;
	int sphereIndex = -1;
	int triangleIndex = -1;
};