#pragma once
#include "glm/glm.hpp"

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
		glm::vec3 center
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
		center(center)
	{
	}

	Intersection()
	{
	}

	~Intersection()
	{
	}

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
};