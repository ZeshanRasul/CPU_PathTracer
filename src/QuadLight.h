#pragma once
#include "../include/glm/glm.hpp"

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
		v0 = a;
		v1 = a + ab;
		v2 = a + ab + ac;
		v3 = a + ac;
	}

	void GetThetaForAllVertices(glm::vec3 r)
	{
		theta0 = glm::acos(glm::dot(glm::normalize(v0 - r), glm::normalize(v1 - r)));
		theta1 = glm::acos(glm::dot(glm::normalize(v1 - r), glm::normalize(v2 - r)));
		theta2 = glm::acos(glm::dot(glm::normalize(v2 - r), glm::normalize(v3 - r)));
		theta3 = glm::acos(glm::dot(glm::normalize(v3 - r), glm::normalize(v0 - r)));
	}

	void GetGammaForAllVertices(glm::vec3 r)
	{
		gamma0 = glm::normalize(glm::cross((v0 - r), (v1 - r)));
		gamma1 = glm::normalize(glm::cross((v1 - r), (v2 - r)));
		gamma2 = glm::normalize(glm::cross((v2 - r), (v3 - r)));
		gamma3 = glm::normalize(glm::cross((v3 - r), (v0 - r)));
	}

	glm::vec3 GetPhi()
	{
		phi = 0.5f * ((theta0 * gamma0) + (theta1 * gamma1) + (theta2 * gamma2) + (theta3 * gamma3));
		return phi;
	}

public:
	glm::vec3 a;
	glm::vec3 ab;
	glm::vec3 ac;
	glm::vec3 intensity;
	glm::vec3 v0;
	glm::vec3 v1;
	glm::vec3 v2;
	glm::vec3 v3;
	glm::vec3 gamma0;
	glm::vec3 gamma1;
	glm::vec3 gamma2;
	glm::vec3 gamma3;
	float theta0;
	float theta1;
	float theta2;
	float theta3;
	glm::vec3 phi;
};