#pragma once

#include "../include/glm/glm.hpp"

class Camera
{
public:
	Camera(glm::vec3 eyePos, glm::vec3 center, glm::vec3 up, float fovY)
		:
		eyePos(eyePos),
		center(center),
		up(up),
		fovY(fovY)
	{
	}

	~Camera()
	{
	}

	glm::vec3 getEyePos() const { return eyePos; }
	glm::vec3 getCenter() const { return center; }
	glm::vec3 getUp() const { return up; }
	float getFovY() const { return fovY; }

private:
	glm::vec3 eyePos, center, up;
	float fovY;
};