#pragma once
#include <glm/glm.hpp>

class QuadLight
{
public:
	QuadLight(glm::vec3 a, glm::vec3 ab, glm::vec3 ac, glm::vec3 intensity)
		:
		a(a),
		ab(ab),
		ac(ac),
		intensity(intensity)
	{
	}

public:
	glm::vec3 a;
	glm::vec3 ab;
	glm::vec3 ac;
	glm::vec3 intensity;
};