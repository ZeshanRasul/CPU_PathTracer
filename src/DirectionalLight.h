#pragma once
#include "glm/glm.hpp"

class DirectionalLight
{
public:
	DirectionalLight(glm::vec3 direction, glm::vec3 colour)
		:
		direction(direction),
		colour(colour)
	{
	}

	glm::vec3 direction;
	glm::vec3 colour;
};