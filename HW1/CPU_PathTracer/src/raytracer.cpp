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

glm::vec3 CheckTriangleIntersection(Triangle* triangle, Ray& ray)
{
	glm::vec3 e1 = triangle->vertex1 - triangle->vertex0;
	glm::vec3 e2 = triangle->vertex2 - triangle->vertex0;
	glm::vec3 s = ray.origin - triangle->vertex0;
	glm::vec3 s1 = glm::cross(ray.direction, e2);
	glm::vec3 s2 = glm::cross(s, e1);

	glm::vec3 tBetaGamma = (1 / glm::dot(s1, e1)) * glm::vec3(glm::dot(s2, e2), glm::dot(s1, s), glm::dot(s2, ray.direction));

	float t = tBetaGamma.x;
	float beta = tBetaGamma.y;
	float gamma = tBetaGamma.z;
	float alpha = 1 - beta - gamma;


	glm::vec3 q = ray.origin + (t * ray.direction);

	glm::vec3 bMinusA = triangle->vertex1 - triangle->vertex0;
	glm::vec3 cMinusA = triangle->vertex2 - triangle->vertex0;

	glm::vec3 cMinusB = triangle->vertex2 - triangle->vertex1;
	glm::vec3 qMinusB = q - triangle->vertex1;

	glm::vec3 aMinusC = triangle->vertex0 - triangle->vertex2;
	glm::vec3 qMinusC = q - triangle->vertex2;

	glm::vec3 qMinusA = q - triangle->vertex0;

	float denominator = (glm::dot(glm::cross(bMinusA, cMinusA), triangle->normalA));

	float alpha2 = (glm::dot(glm::cross(cMinusB, qMinusB), triangle->normalA)) / denominator;
	float beta2 = (glm::dot(glm::cross(aMinusC, qMinusC), triangle->normalA)) / denominator;
	float gamma2 = (glm::dot(glm::cross(bMinusA, qMinusA), triangle->normalA)) / denominator;

	if (beta2 < 0 || gamma2 < 0 || beta2 + gamma2 > 1 || beta2 > 1 || gamma2 > 1)
	{
		glm::vec3 nullT(INFINITY, 0, 0);
		return nullT;
	}

	glm::vec3 tBetaGamma2(t, beta, gamma);

	triangle->SetNormal(glm::normalize((alpha2 * triangle->normalA) + (beta2 * triangle->normalB) + (gamma2 * triangle->normalC)));
	return tBetaGamma2;
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
	Triangle* hitTri = NULL;
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
			hitObjectIsSphere = true;
		}
	}

	std::vector<Triangle*> sceneTriangles = scene->GetTriangles();

	for (Triangle* tri : sceneTriangles)
	{
		tBetaGamma2 = CheckTriangleIntersection(tri, ray);
		tTriangle = tBetaGamma2.x;
		float beta2 = tBetaGamma2.y;
		float gamma2 = tBetaGamma2.z;
		float alpha2 = 1 - beta2 - gamma2;

		if (tTriangle < minDist && tTriangle > 0)
		{
			hitObjectDiffuse = tri->diffuse;
			hitObjectSpecular = tri->specular;
			hitObjectEmission = tri->emission;
			hitObjectAmbient = tri->ambient;
			hitObjectShininess = tri->shininess;
			hitObjectIsSphere = false;


			hitObjectNormal = glm::normalize((alpha2 * tri->normalA) + (beta2 * tri->normalB) + (gamma2 * tri->normalC));

			intersectionPoint = ray.origin + ray.direction * tTriangle;

			minDist = tTriangle;
			didHit = true;
			t = tTriangle;
			hitTri = tri;

		}
	}

	if (didHit)
	{

		if (hitObjectIsSphere)
		{
			intersectionPoint = ray.origin + (glm::normalize(ray.direction) * tSphere);
			glm::vec3 sphereNormal = glm::normalize(intersectionPoint - hitSphere->center);
			hitSphere->SetNormal(sphereNormal);
			Intersection* intersection = new Intersection(didHit, intersectionPoint, hitSphere->diffuse, hitSphere->specular, hitSphere->emission, hitSphere->shininess, hitSphere->ambient, hitSphere->normal, hitSphere->center);
			return intersection;
		}
		else
		{
			Intersection* intersection = new Intersection(didHit, intersectionPoint, hitObjectDiffuse, hitObjectSpecular, hitObjectEmission, hitObjectShininess, hitObjectAmbient, hitTri->normal, glm::vec3(0, 0, 0));
			return intersection;
		}
	}
	else
	{
		Intersection* intersection = new Intersection(false, intersectionPoint, hitObjectDiffuse, hitObjectSpecular, hitObjectEmission, hitObjectShininess, hitObjectAmbient, hitObjectNormal, glm::vec3(0, 0, 0));
		return intersection;
	}
}

glm::vec3 FindColor(Intersection* intersection, Scene* scene, Camera* camera)
{
	glm::vec3 finalCol = glm::vec3(0.0f);
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


			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(scene, shadowRay);
			if (shadowIntersection->didHit)
			{
				finalCol += intersection->hitObjectAmbient;
				continue;
			}

			finalCol += lambert + phong + intersection->hitObjectAmbient;
		}

		return finalCol;
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

	glm::vec3 eyePos(0, 0, 75);
	glm::vec3 center(0, 0, -1);
	glm::vec3 up(0, 1, 0);

	Camera cam(eyePos, center, up, glm::radians(45.0f));

	Scene* scene = new Scene();
	Sphere* sphere = new Sphere(glm::vec3(2.0, 0, -0), 1.3f, glm::vec3(1, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0, 0, 0), 32.0f, glm::vec3(0.3f, 0.3f, 0.3f));
	scene->AddSphere(sphere);

	Sphere* sphere2 = new Sphere(glm::vec3(-7.0f, 1.0f, 0.0f), 2.5f, glm::vec3(0, 0, 1), glm::vec3(1, 1, 1), glm::vec3(0, 0, 0), 32.0f, glm::vec3(0.3f, 0.3f, 0.3f));
	scene->AddSphere(sphere2);

	glm::vec3 col = glm::vec3(1.0f, 0.0f, 1.0f);

	DirectionalLight* dirLight = new DirectionalLight(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f));
	scene->AddDirectionalLight(dirLight);
	//DirectionalLight* dirLight2 = new DirectionalLight(glm::vec3(0.1f, -0.8f, 0.3f), glm::vec3(0.4f, 0.4f, 0.4f));
	//scene->AddDirectionalLight(dirLight2);

	float triWidth = 100.0f;
	float triHeight = 10.00f;
	float triDepth = 200.0f;
	float triCenter = 0.00f;

	glm::vec3 vert0(-triWidth, -triHeight, -triDepth);
	glm::vec3 vert1(-triWidth, +triHeight, -triDepth);
	glm::vec3 vert2(+triWidth, +triHeight, -triDepth);
	glm::vec3 vert3(+triWidth, -triHeight, -triDepth);
	glm::vec3 vert4(-triWidth, -triHeight, +triDepth);
	glm::vec3 vert5(-triWidth, +triHeight, +triDepth);
	glm::vec3 vert6(+triWidth, +triHeight, +triDepth);
	glm::vec3 vert7(+triWidth, -triHeight, +triDepth);

	Triangle* tri0 = new Triangle(vert0, vert4, vert7, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri1 = new Triangle(vert0, vert7, vert3, glm::vec3(1.0f, 0.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri2 = new Triangle(vert1, vert5, vert6, glm::vec3(0.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(1.0f, 1.00f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri3 = new Triangle(vert1, vert6, vert2, glm::vec3(0.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(1.00f, 1.00f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri4 = new Triangle(vert3, vert2, vert6, glm::vec3(0.0f, 0.0f, 1.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri5 = new Triangle(vert3, vert6, vert7, glm::vec3(0.0f, 0.0f, 1.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri6 = new Triangle(vert0, vert5, vert1, glm::vec3(1.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.00f, 1.00f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri7 = new Triangle(vert0, vert4, vert5, glm::vec3(1.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.00f, 1.00f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri8 = new Triangle(vert0, vert1, vert2, glm::vec3(0.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri9 = new Triangle(vert0, vert2, vert3, glm::vec3(0.0f, 1.0f, 0.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri10 = new Triangle(vert4, vert7, vert6, glm::vec3(0.0f, 1.0f, 1.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));
	Triangle* tri11 = new Triangle(vert4, vert6, vert5, glm::vec3(0.0f, 1.0f, 1.f), glm::vec3(0.00, 0.00, 0.00f), glm::vec3(0.15f, 0.05f, 0.0f), 1.00f, glm::vec3(0.3f, 0.3f, 0.3f));


	//// -Y
	//scene->AddTriangle(tri0);
	//scene->AddTriangle(tri1);

	// +Y
	scene->AddTriangle(tri2);
	scene->AddTriangle(tri3);

	//// +X
	//scene->AddTriangle(tri4);
	//scene->AddTriangle(tri5);

	//// -X
	//scene->AddTriangle(tri6);
	//scene->AddTriangle(tri7);
	//// -Z
	//scene->AddTriangle(tri8);
	//scene->AddTriangle(tri9);


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