#include "Freeimage.h"
#include <string>
#include <cstdlib> 

#include "Camera.h"
#include "Ray.h"
#include "Intersection.h"
#include "Scene.h"

const int IMAGE_WIDTH = 240;
const int IMAGE_HEIGHT = 120;

Ray ShootRay(const Camera& cam, const int i, const int j, const int width, const int height)
{
	glm::vec3 a, b, u, v, w;

	a = cam.getEyePos() - cam.getCenter();
	b = cam.getUp();

	w = glm::normalize(a);
	u = (glm::normalize(glm::cross(b, w)));
	v = glm::cross(w, u);

	float fovX = 2 * atan((height * tan(cam.getFovY() / 2.0f)) / width);
	float alpha = (tan(cam.getFovY() / 2.0f) * ((float(i) - (float(width) / 2.0f)) / (float(width) / 2.0f)));
	float beta = tan(fovX / 2.0f) * (((float(height) / 2.0f) - float(j)) / (float(height) / 2.0f));

	glm::vec3 direction = glm::normalize((alpha * u) + (beta * v) - w);
	glm::vec3 origin = cam.getEyePos();
	Ray ray(origin, direction);

	return ray;

}

float CheckSphereIntersection(Sphere* sphere, Ray& ray)
{
	float a = glm::dot(ray.direction, ray.direction);
	float b = 2 * (glm::dot(ray.direction, (ray.origin - sphere->center)));
	float c = glm::dot((ray.origin - sphere->center), (ray.origin - sphere->center)) - (sphere->radius * sphere->radius);

//	float discriminant = (b * b) - (4 * a * c);
	float discriminant = (glm::dot(ray.direction, (ray.origin - sphere->center))) * (glm::dot(ray.direction, (ray.origin - sphere->center))) - ((glm::dot(ray.origin - sphere->center, ray.origin - sphere->center)) - (sphere->radius * sphere->radius));

	int roots = 0;
	bool pickPositiveRoot = false;
	bool pickSmallerRoot = false;

	if (discriminant > 0)
	{
		roots = 2;
		pickSmallerRoot = true;
	}
	else if (discriminant < 0)
	{
		return INFINITY;
	}
	else if (discriminant == 0)
	{
		roots = 1;

	}

	float t1 = (-b + glm::sqrt(discriminant)) / (2 * a);
	float t2 = (-b - glm::sqrt(discriminant)) / (2 * a);

	if (t1 < 0 && t2 > 0)
		return t2;

	if (t2 < 0 && t1 > 0)
		return t1;

	if (pickSmallerRoot)
		return t1 < t2 ? t1 : t2;

	if (t2 > 0 && t1 > 0)
		return t1 > t2 ? t1 : t2;

	return INFINITY;
}

Intersection* FindIntersection(Scene* scene, Ray& ray)
{
	float minDist = INFINITY;
	Sphere* hitSphere = NULL;
	bool didHit = false;
	float t = minDist;
	float tSphere = minDist;
	float tTriangle = minDist;
	glm::vec3 hitObjectDiffuse = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectSpecular = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectEmission = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectAmbient = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectNormal = glm::vec3(0, 0, 0);
	float hitObjectShininess = 0.0f;
	glm::vec3 tBetaGamma2;
	glm::vec3 intersectionPoint(INFINITY, INFINITY, INFINITY);
	glm::vec3 center(0, 0, 0);
	bool hitObjectIsSphere = false;
	glm::vec3 reflectionNormal(0, 0, 0);

	std::vector<Sphere*> sceneSpheres = scene->GetSpheres();

	for (Sphere* sphere : sceneSpheres)
	{
		t = CheckSphereIntersection(sphere, ray);
		if (t < minDist)
		{
			minDist = t;
			didHit = true;
			hitSphere = sphere;
			tSphere = t;
		}
	}

	if (didHit)
	{
		intersectionPoint = ray.origin + (glm::normalize(ray.direction) * tSphere);
		glm::vec3 sphereNormal = glm::normalize(intersectionPoint - hitSphere->center);
		hitSphere->SetNormal(sphereNormal);
		Intersection* intersection = new Intersection(didHit, intersectionPoint, hitSphere->diffuse, hitSphere->specular, hitSphere->emission, hitSphere->shininess, hitSphere->ambient, hitSphere->normal, hitSphere->center);
		return intersection;
	}
	else
	{
		Intersection* intersection = new Intersection(false, intersectionPoint, hitObjectDiffuse, hitObjectSpecular, hitObjectEmission, hitObjectShininess, hitObjectAmbient, hitObjectNormal, glm::vec3(0, 0, 0));
		return intersection;
	}
}

glm::vec3 FindColor(Intersection* intersection, Scene* scene, Camera* camera)
{
	if (intersection->didHit)
	{
		std::vector<DirectionalLight*> dirLights = scene->GetDirLights();
		for (auto& dirLight : dirLights)
		{
			glm::vec3 eyeDir = glm::normalize(camera->getEyePos() - intersection->intersectionPoint);
			glm::vec3 normalizedLightDirection = glm::normalize(-dirLight->direction);

			float nDotL = glm::dot(intersection->hitObjectNormal, normalizedLightDirection);
			// No attenuation for now
			glm::vec3 lambert = intersection->hitObjectDiffuse * dirLight->colour * std::max(nDotL, 0.0f);

			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection->hitObjectNormal, halfVec);
			glm::vec3 phong = intersection->hitObjectSpecular * dirLight->colour * pow(std::max(nDotH, 0.0f), intersection->hitObjectShininess);


			return lambert + phong + intersection->hitObjectAmbient;
		}
	}
	else
	{
		return glm::vec3(1.0f, 0.0f, 1.0f);
	}
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

	Camera cam(eyePos, center, up, glm::radians(45.0f));

	Scene* scene = new Scene();
	Sphere* sphere = new Sphere(glm::vec3(0, 0, -15), 1.0f, glm::vec3(1, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0, 0, 0), 32.0f, glm::vec3(0.1f, 0.1f, 0.1f));
	scene->AddSphere(sphere);
	
	Sphere* sphere2 = new Sphere(glm::vec3(-3.0f, 1.0f, -15.0f), 0.5f, glm::vec3(0, 0, 1), glm::vec3(1, 1, 1), glm::vec3(0, 0, 0), 32.0f, glm::vec3(0.1f, 0.1f, 0.1f));
	scene->AddSphere(sphere2);

	glm::vec3 col = glm::vec3(1.0f, 0.0f, 1.0f);

	DirectionalLight* dirLight = new DirectionalLight(glm::vec3(0.1f, 1.0f, -0.3f), glm::vec3(1.0f, 1.0f, 1.0f));
	scene->AddDirectionalLight(dirLight);

	for (int y = 0; y < IMAGE_HEIGHT; y++)
	{
		for (int x = 0; x < IMAGE_WIDTH; x++)
		{
			Ray ray = ShootRay(cam, x, y, IMAGE_WIDTH, IMAGE_HEIGHT);
			Intersection* intersection = FindIntersection(scene, ray);
			col = FindColor(intersection, scene, &cam);
			int idx = (y * IMAGE_WIDTH + x) * 3;
			pixels[idx + 0] = std::min(col.b * 255, 255.0f);
			pixels[idx + 1] = std::min(col.g * 255, 255.0f);
			pixels[idx + 2] = std::min(col.r * 255, 255.0f);
		}

	}
	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}