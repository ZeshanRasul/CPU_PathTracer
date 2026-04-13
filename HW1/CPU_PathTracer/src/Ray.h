#pragma once

#include "glm/glm.hpp"

class Ray
{
public:
	Ray(glm::vec3 origin, glm::vec3 direction)
		:
		origin(origin),
		direction(direction),
		bounces(1)
	{
	}

	Ray()
		:
		bounces(1)
	{
	}

	~Ray()
	{
	}

	glm::vec3 origin;
	glm::vec3 direction;
	int bounces;
};