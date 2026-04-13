#include "Freeimage.h"
#include <string>
#include <cstdlib> 

#include "Camera.h"
#include "Ray.h"

const int IMAGE_WIDTH = 240;
const int IMAGE_HEIGHT = 120;

Ray ShootRay(const Camera& cam, const int i, const int j, const int width, const int height)
{
	glm::vec3 a, b, u, v, w;

	a = cam.getEyePos() - cam.getCenter();
	b = cam.getUp();

	w = a / glm::normalize(a);
	u = (glm::cross(b, w)) / (glm::normalize(glm::cross(b, w)));
	v = glm::cross(w, u);

	float fovX = 2 * atan((height * tan(cam.getFovY() / 2)) / width);
	float alpha = (tan(cam.getFovY() / 2) * ((float(i) - (float(width) / 2)) / (float(width) / 2)));
	float beta = tan(fovX / 2) * (((float(height) / 2) - float(j)) / (float(height) / 2));

	glm::vec3 direction = normalize((alpha * u) + (beta * v) - w);
	glm::vec3 origin = cam.getEyePos();
	Ray ray(origin, direction);

	return ray;

}

int main()
{
	std::string fname = "outfile.png";
	FreeImage_Initialise();

	BYTE* pixels = (BYTE*)malloc(sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);
	if (!pixels) {
		FreeImage_DeInitialise();
		return 1;
	}
	memset(pixels, 0, sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);

	glm::vec3 eyePos(0, 0, 0);
	glm::vec3 center(0, 0, -1);
	glm::vec3 up(0, 1, 0);

	Camera cam(eyePos, center, up, 90.0f);

	for (int y = 0; y < IMAGE_HEIGHT; y++)
	{
		for (int x = 0; x < IMAGE_WIDTH; x++)
		{
			int idx = (y * IMAGE_WIDTH + x) * 3;
			pixels[idx + 0] = 255;
			pixels[idx + 1] = 255;
			pixels[idx + 2] = 255;
		}

	}
	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}