#pragma once
#include "glm/glm.hpp"

class PointLight
{
public:
	PointLight(glm::vec3 position, glm::vec3 colour)
		:
		position(position),
		colour(colour)
	{
	}

	glm::vec3 position;
	glm::vec3 colour;
};