#pragma once
#include "../include/glm/glm.hpp"
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
		bool isAreaLight,
		std::string brdf,
		int lightIndex = -1,
		float hitObjectRoughness = 0.0f,
		bool hitHasTexture = false,
		texture* hitObjectTexture = nullptr,
		bool isShadowRay = false
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
		isLight(isAreaLight),
		brdf(brdf),
		hitObjectRoughness(hitObjectRoughness),
		hitHasTexture(hitHasTexture),
		hitObjectTexture(hitObjectTexture),
		isShadowRay(isShadowRay),
		lightIndex(lightIndex)
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
	bool isLight;
	int sphereIndex = -1;
	int triangleIndex = -1;
	bool isShadowRay = false;
	std::string brdf = "phong";
	float hitObjectRoughness = 0.0f;
	int lightIndex = -1;
};