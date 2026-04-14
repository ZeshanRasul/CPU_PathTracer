#include "Freeimage.h"
#include <string>
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include <sstream>
#include "glm/gtc/matrix_transform.hpp"

#include "Camera.h"
#include "Ray.h"
#include "Intersection.h"
#include "Scene.h"


int bounces = 0;
int maxDepth;
glm::mat4 modelStack = glm::mat4(1.0f);
glm::mat4 prevMatrix = glm::mat4(1.0f);
glm::mat4 invModelStack = glm::mat4(1.0f);
glm::mat4 prevInvMatrix = glm::mat4(1.0f);
glm::mat4 normalMatrix = glm::mat4(1.0f);
glm::mat4 scaleMatrix = glm::mat4(1.0f);
std::vector<glm::mat4> transformStack = { glm::mat4(1.0f) };


Ray ShootRay(const Camera& cam, const int i, const int j, const int width, const int height)
{
	glm::vec3 w = glm::normalize(cam.getEyePos() - cam.getCenter());
	glm::vec3 u = glm::normalize(glm::cross(cam.getUp(), w));
	glm::vec3 v = glm::cross(w, u);

	float fovY = cam.getFovY();
	float aspect = static_cast<float>(width) / static_cast<float>(height);
	float fovX = 2.0f * atan(aspect * tan(fovY * 0.5f));

	float alpha = tan(fovX * 0.5f) * ((2.0f * (static_cast<float>(i) + 0.5f) / static_cast<float>(width)) - 1.0f);
	float beta = tan(fovY * 0.5f) * (1.0f - (2.0f * (static_cast<float>(j) + 0.5f) / static_cast<float>(height)));

	glm::vec3 direction = glm::normalize(alpha * u + beta * v - w);
	return Ray(cam.getEyePos(), direction);
}

glm::vec3 CheckTriangleIntersection(Triangle* triangle, Ray& ray)
{
	const float epsilon = 1e-6f;

	glm::vec3 e1 = triangle->vertex1 - triangle->vertex0;
	glm::vec3 e2 = triangle->vertex2 - triangle->vertex0;

	glm::vec3 pvec = glm::cross(ray.direction, e2);
	float det = glm::dot(e1, pvec);

	if (fabs(det) < epsilon)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	float invDet = 1.0f / det;
	glm::vec3 tvec = ray.origin - triangle->vertex0;

	float beta = glm::dot(tvec, pvec) * invDet;
	if (beta < 0.0f || beta > 1.0f)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	glm::vec3 qvec = glm::cross(tvec, e1);
	float gamma = glm::dot(ray.direction, qvec) * invDet;
	if (gamma < 0.0f || beta + gamma > 1.0f)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	float t = glm::dot(e2, qvec) * invDet;
	if (t <= epsilon)
	{
		return glm::vec3(INFINITY, 0.0f, 0.0f);
	}

	glm::vec3 faceNormal = glm::normalize(glm::cross(e1, e2));
	triangle->SetNormal(faceNormal);

	return glm::vec3(t, beta, gamma);
}

float CheckSphereIntersection(Sphere* sphere, Ray& ray)
{
	glm::vec3 oc = ray.origin - sphere->center;

	float a = glm::dot(ray.direction, ray.direction);
	float b = 2.0f * glm::dot(ray.direction, oc);
	float c = glm::dot(oc, oc) - sphere->radius * sphere->radius;

	float discriminant = b * b - 4.0f * a * c;

	if (discriminant < 0.0f)
	{
		return INFINITY;
	}

	float sqrtDiscriminant = glm::sqrt(discriminant);
	float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
	float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

	if (t1 > 0.0f)
	{
		return t1;
	}

	if (t2 > 0.0f)
	{
		return t2;
	}

	return INFINITY;
}

Intersection* FindIntersection(Scene* scene, Ray& ray)
{
	float minDist = INFINITY;
	Sphere* hitSphere = NULL;
	Triangle* hitTri = NULL;
	bool didHit = false;
	float worldt = minDist;
	float t = minDist;
	float tSphere = minDist;
	float tTriangle = minDist;
	glm::vec3 hitObjectDiffuse = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectSpecular = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectEmission = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectAmbient = glm::vec3(0, 0, 0);
	glm::vec3 hitObjectNormal = glm::vec3(0, 0, 0);
	glm::vec3 sphereLocalNormal = glm::vec3(0, 0, 0);
	glm::vec3 sphereWorldNormal = glm::vec3(0, 0, 0);
	float hitObjectShininess = 0.0f;
	glm::vec3 tBetaGamma2;
	glm::vec3 intersectionPoint(INFINITY, INFINITY, INFINITY);
	bool hitObjectIsSphere = false;
	float hitObjectIOR = 1.0f;
	bool hitHasTexture = false;
	texture* hitObjectTexture = NULL;

	std::vector<Sphere*> sceneSpheres = scene->GetSpheres();

	for (Sphere* sphere : sceneSpheres)
	{
		glm::mat4 invTransform = glm::inverse(sphere->transform);

		Ray transformedRay(
			glm::vec3(invTransform * glm::vec4(ray.origin, 1.0f)),
			glm::vec3(invTransform * glm::vec4(ray.direction, 0.0f)));

		t = CheckSphereIntersection(sphere, transformedRay);

		if (t > 0.0f && t < INFINITY)
		{
			glm::vec3 localHitPoint = transformedRay.origin + transformedRay.direction * t;
			glm::vec3 worldHitPoint = glm::vec3(sphere->transform * glm::vec4(localHitPoint, 1.0f));
			float worldDist = glm::length(worldHitPoint - ray.origin);

			if (worldDist < minDist)
			{
				minDist = worldDist;
				didHit = true;
				hitSphere = sphere;
				tSphere = t;
				hitObjectIsSphere = true;
				hitObjectAmbient = sphere->ambient;
				hitObjectDiffuse = sphere->diffuse;
				hitObjectSpecular = sphere->specular;
				hitObjectEmission = sphere->emission;
				hitObjectShininess = sphere->shininess;
				hitObjectIOR = sphere->ior;
				hitHasTexture = sphere->matTexture != NULL;
				hitObjectTexture = sphere->matTexture;
			}
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
			hitObjectNormal = tri->normal;
			intersectionPoint = ray.origin + ray.direction * tTriangle;
			minDist = tTriangle;
			didHit = true;
			hitTri = tri;

		}
	}

	if (didHit)
	{
		if (hitObjectIsSphere)
		{
			glm::mat4 invTransform = glm::inverse(hitSphere->transform);

			Ray transformedRay(
				glm::vec3(invTransform * glm::vec4(ray.origin, 1.0f)),
				glm::vec3(invTransform * glm::vec4(ray.direction, 0.0f)));

			glm::vec3 localHitPoint = transformedRay.origin + transformedRay.direction * tSphere;
			glm::vec3 localNormal = glm::normalize(localHitPoint - hitSphere->center);

			intersectionPoint = glm::vec3(hitSphere->transform * glm::vec4(localHitPoint, 1.0f));

			glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(hitSphere->transform)));
			hitObjectNormal = glm::normalize(normalMat * localNormal);

			return new Intersection(
				true,
				intersectionPoint,
				hitObjectDiffuse,
				hitObjectSpecular,
				hitObjectEmission,
				hitObjectShininess,
				hitObjectAmbient,
				hitObjectNormal,
				glm::vec3(hitSphere->transform * glm::vec4(hitSphere->center, 1.0f)),
				hitObjectIOR,
				hitHasTexture,
				hitObjectTexture);
		}
		else
		{

			return new Intersection(
				true,
				intersectionPoint,
				hitObjectDiffuse,
				hitObjectSpecular,
				hitObjectEmission,
				hitObjectShininess,
				hitObjectAmbient,
				hitObjectNormal,
				glm::vec3(0, 0, 0),
				hitObjectIOR,
				hitHasTexture,
				NULL);
		}
	}

	return new Intersection(
		false,
		intersectionPoint,
		hitObjectDiffuse,
		hitObjectSpecular,
		hitObjectEmission,
		hitObjectShininess,
		hitObjectAmbient,
		hitObjectNormal,
		glm::vec3(0, 0, 0),
		1.0f,
		hitHasTexture,
		NULL);
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

			if (intersection->hitHasTexture)
			{
				intersection->hitObjectDiffuse = intersection->hitObjectTexture->value(0, 0, intersection->intersectionPoint);
			}
			glm::vec3 lambert = intersection->hitObjectDiffuse * dirLight->colour * std::max(nDotL, 0.0f);

			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection->hitObjectNormal, halfVec);
			glm::vec3 phong = intersection->hitObjectSpecular * dirLight->colour * pow(std::max(nDotH, 0.0f), intersection->hitObjectShininess);

			for (int i = 0; i < maxDepth; i++)
			{
				bounces++;
				if (bounces >= maxDepth)
				{
					break;
				}
				glm::vec3 reflectDir = glm::reflect(-normalizedLightDirection, intersection->hitObjectNormal);
				glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
				Ray mirrorRay(origin, reflectDir);
				Intersection* mirrorIntersection = FindIntersection(scene, mirrorRay);
				finalCol += FindColor(mirrorIntersection, scene, camera) * intersection->hitObjectSpecular;

				if (intersection->hitObjectIOR > 1.0f)
				{
					glm::vec3 refractDir = glm::refract(-normalizedLightDirection, intersection->hitObjectNormal, 1.0f / intersection->hitObjectIOR);
					Ray refractRay(origin, refractDir);
					Intersection* refractIntersection = FindIntersection(scene, refractRay);
					finalCol += FindColor(refractIntersection, scene, camera) * intersection->hitObjectSpecular;
				}
			}
			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(scene, shadowRay);
			if (shadowIntersection->didHit)
			{
				//finalCol += intersection->hitObjectAmbient;
				delete shadowIntersection;
				continue;
			}

			finalCol += lambert + phong;
		}

		std::vector<PointLight*> pointLights = scene->GetPointLights();

		for (auto& pointLight : pointLights)
		{
			glm::vec3 lightDir = pointLight->position - intersection->intersectionPoint;
			float distanceToLight = glm::length(lightDir);
			glm::vec3 eyeDir = glm::normalize(camera->getEyePos() - intersection->intersectionPoint);
			glm::vec3 normalizedLightDirection = glm::normalize(lightDir);
			float nDotL = glm::dot(intersection->hitObjectNormal, normalizedLightDirection);
			// No attenuation for now
			if (intersection->hitHasTexture)
			{
				intersection->hitObjectDiffuse = intersection->hitObjectTexture->value(0, 0, intersection->intersectionPoint);
			}
			glm::vec3 lambert = intersection->hitObjectDiffuse * pointLight->colour * std::max(nDotL, 0.0f);
			glm::vec3 halfVec = glm::normalize(normalizedLightDirection + eyeDir);
			float nDotH = glm::dot(intersection->hitObjectNormal, halfVec);
			glm::vec3 phong = intersection->hitObjectSpecular * pointLight->colour * pow(std::max(nDotH, 0.0f), intersection->hitObjectShininess);

			for (int i = 0; i < maxDepth; i++)
			{
				bounces++;

				if (bounces >= maxDepth)
				{
					break;
				}
				glm::vec3 reflectDir = glm::reflect(-normalizedLightDirection, intersection->hitObjectNormal);
				glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
				Ray mirrorRay(origin, reflectDir);
				Intersection* mirrorIntersection = FindIntersection(scene, mirrorRay);
				finalCol += FindColor(mirrorIntersection, scene, camera) * intersection->hitObjectSpecular;

				if (intersection->hitObjectIOR > 1.0f)
				{
					glm::vec3 refractDir = glm::refract(-normalizedLightDirection, intersection->hitObjectNormal, 1.0f / intersection->hitObjectIOR);
					Ray refractRay(origin, refractDir);
					Intersection* refractIntersection = FindIntersection(scene, refractRay);
					finalCol += FindColor(refractIntersection, scene, camera) * intersection->hitObjectSpecular;
				}
			}
			// Shadow Ray
			glm::vec3 dir = normalizedLightDirection;
			glm::vec3 origin = intersection->intersectionPoint + (intersection->hitObjectNormal * 0.001f);
			Ray shadowRay(origin, dir);
			Intersection* shadowIntersection = FindIntersection(scene, shadowRay);
			if (shadowIntersection->didHit)
			{
				float distanceToOccluder = glm::length(shadowIntersection->intersectionPoint - intersection->intersectionPoint);
				if (distanceToOccluder < distanceToLight)
				{
					delete shadowIntersection;
					continue;
				}
			}

			finalCol += lambert + phong;
		}
		return finalCol + intersection->hitObjectEmission + intersection->hitObjectAmbient;
	}
	else
	{
		return finalCol = glm::vec3(0.0f, 0.0f, 0.0f);
	}
}


struct Vertex
{
	glm::vec3 position;
};

//struct Tri
//{
//	Vertex v0;
//	Vertex v1;
//	Vertex v2;
//
//	Tri(Vertex v0, Vertex v1, Vertex v2)
//		:
//		v0(v0),
//		v1(v1),
//		v2(v2)
//	{
//	}
//};

//struct Sphere
//	{
//	glm::vec3 center;
//	float radius;
//	Sphere(glm::vec3 center, float radius)
//		:
//		center(center),
//		radius(radius)
//	{
//	}
//};

std::vector<Vertex> verts;
//std::vector<Tri> triangles;
//std::vector<Sphere> spheres;


int main() {
	std::string fname = "outfile.png";
	FreeImage_Initialise();

	glm::vec3 eyePos, center, up;
	int width;
	int height;
	int eyeX, eyeY, eyeZ;
	int centerX, centerY, centerZ;
	int upX, upY, upZ;
	float fovY;
	float ambientR, ambientG, ambientB;
	float diffuseR, diffuseG, diffuseB;
	float specularR, specularG, specularB;
	float shininess;
	float emissionR, emissionG, emissionB;
	float dirLightX, dirLightY, dirLightZ;
	float dirLightR, dirLightG, dirLightB;
	float sphereX, sphereY, sphereZ, sphereRadius;
	int maxVerts;
	int maxVertNorms;
	Scene* scene = new Scene();


	std::ifstream file("C:/dev/CSE168x/HW1/CPU_PathTracer/Release/scene4-diffuse.test");
	std::string line;

	while (std::getline(file, line))
	{
		// Remove null characters from the line
		line.erase(std::remove(line.begin(), line.end(), '\0'), line.end());

		if (line.empty())
			continue;

		// Process clean line

		if (line[0] == '#') {
			std::cout << line << std::endl;
			continue;
		}

		std::istringstream iss(line);
		std::string cmd;
		iss >> cmd;

		if (cmd == "popTransform")
		{
			std::cout << line << std::endl;
			modelStack = prevMatrix;

			if (transformStack.size() > 1)
			{
				transformStack.pop_back();
			}
		}

		if (cmd == "pushTransform")
		{
			std::cout << line << std::endl;
			transformStack.push_back(transformStack.back());
		}

		if (cmd == "translate")
		{
			std::cout << line << std::endl;
			float x, y, z;
			iss >> x >> y >> z;
			transformStack.back() = glm::translate(transformStack.back(), glm::vec3(x, y, z));
		}

		if (cmd == "scale")
		{
			std::cout << line << std::endl;
			float x, y, z;
			iss >> x >> y >> z;
			transformStack.back() = glm::scale(transformStack.back(), glm::vec3(x, y, z));
			scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(x, y, z));
		}

		if (cmd == "rotate")
		{
			std::cout << line << std::endl;
			float x, y, z, angle;
			iss >> x >> y >> z >> angle;

			glm::vec3 axis(x, y, z);
			float angleRadians = glm::radians(angle);

			transformStack.back() = glm::rotate(transformStack.back(), angleRadians, axis);
		}

		if (cmd == "size")
		{
			int w, h;
			iss >> w >> h;
			width = w;
			height = h;
		}

		if (cmd == "maxDepth")
		{
			int d;
			iss >> d;
			maxDepth = d;
		}

		if (cmd == "output")
		{
			std::string outputFile;
			iss >> outputFile;
			fname = outputFile;
		}

		if (cmd == "camera") {
			std::cout << line << std::endl;
			iss >> eyeX >> eyeY >> eyeZ >> centerX >> centerY >> centerZ >> upX >> upY >> upZ >> fovY;
		}

		if (cmd == "sphere")
		{
			std::cout << line << std::endl;
			iss >> sphereX >> sphereY >> sphereZ >> sphereRadius;
			scene->AddSphere(new Sphere(glm::vec3(sphereX, sphereY, sphereZ), sphereRadius, glm::vec3(diffuseR, diffuseG, diffuseB), glm::vec3(specularR, specularG, specularB), glm::vec3(emissionR, emissionG, emissionB), shininess, glm::vec3(ambientR, ambientG, ambientB), 1.0f, transformStack.back()));
		}

		if (cmd == "maxverts")
		{
			std::cout << line << std::endl;
			iss >> cmd >> maxVerts;
		}

		if (cmd == "maxvertnorms")
		{
			std::cout << line << std::endl;
			iss >> cmd >> maxVertNorms;
		}

		if (cmd == "vertex")
		{
			std::cout << line << std::endl;
			float vertX, vertY, vertZ;
			iss >> vertX >> vertY >> vertZ;
			verts.push_back(Vertex{ glm::vec3(vertX, vertY, vertZ) });
		}

		if (cmd == "vertexnormal")
		{
			std::cout << line << std::endl;
		}

		if (cmd == "tri")
		{
			std::cout << line << std::endl;
			int v0, v1, v2;
			iss >> v0 >> v1 >> v2;
			/*	v0 = v0 - 1;
				v1 = v1 - 1;
				v2 = v2 - 1;*/
			scene->AddTriangle(new Triangle(transformStack.back() * glm::vec4(verts[v0].position, 1.0f), transformStack.back() * glm::vec4(verts[v1].position, 1.0f), transformStack.back() * glm::vec4(verts[v2].position, 1.0f), glm::vec3(diffuseR, diffuseG, diffuseB), glm::vec3(specularR, specularG, specularB), glm::vec3(emissionR, emissionG, emissionB), shininess, glm::vec3(ambientR, ambientG, ambientB), NULL));
		}

		if (cmd == "trinormal")
		{
			std::cout << line << std::endl;
		}

		if (cmd == "directional")
		{
			std::cout << line << std::endl;
			iss >> dirLightX >> dirLightY >> dirLightZ >> dirLightR >> dirLightG >> dirLightB;
			scene->AddDirectionalLight(new DirectionalLight(glm::vec3(dirLightX, dirLightY, dirLightZ), glm::vec3(dirLightR, dirLightG, dirLightB)));
		}

		if (cmd == "point")
		{
			std::cout << line << std::endl;
			float x, y, z, r, g, b;
			iss >> x >> y >> z >> r >> g >> b;
			scene->AddPointLight(new PointLight(glm::vec3(x, y, z), glm::vec3(r, g, b)));
		}

		if (cmd == "attenuation const linear quadratic")
		{
			std::cout << line << std::endl;
		}

		if (cmd == "ambient")
		{
			iss >> ambientR >> ambientG >> ambientB;
		}

		if (cmd == "diffuse")
		{
			std::cout << line << std::endl;
			iss >> diffuseR >> diffuseG >> diffuseB;
		}

		if (cmd == "specular")
		{
			std::cout << line << std::endl;
			iss >> specularR >> specularG >> specularB;
		}

		if (cmd == "shininess")
		{
			std::cout << line << std::endl;
			iss >> shininess;
		}

		if (cmd == "emission")
		{
			std::cout << line << std::endl;
			iss >> emissionR >> emissionG >> emissionB;
		}
	}

	Camera cam(glm::vec3(eyeX, eyeY, eyeZ), glm::vec3(centerX, centerY, centerZ), glm::vec3(upX, upY, upZ), glm::radians(fovY));

	const int IMAGE_WIDTH = width;
	const int IMAGE_HEIGHT = height;

	BYTE* pixels = (BYTE*)malloc(sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);
	if (!pixels) {
		FreeImage_DeInitialise();
		return 1;
	}
	memset(pixels, 0, sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);

	glm::vec3 col = glm::vec3(0.0f, 0.0f, 0.0f);

	for (int y = 0; y < IMAGE_HEIGHT; y++)
	{
		for (int x = 0; x < IMAGE_WIDTH; x++)
		{
			bounces = 0;
			Ray ray = ShootRay(cam, x, y, IMAGE_WIDTH, IMAGE_HEIGHT);
			Intersection* intersection = FindIntersection(scene, ray);
			col = FindColor(intersection, scene, &cam);
			int idx = (y * IMAGE_WIDTH + x) * 3;
			pixels[idx + 0] = std::min(col.b * 255, 255.0f);
			pixels[idx + 1] = std::min(col.g * 255, 255.0f);
			pixels[idx + 2] = std::min(col.r * 255, 255.0f);
			//std::cout << "Pixel (" << x << ", " << y << ") of total (" << IMAGE_WIDTH << ", " << IMAGE_HEIGHT << ")" << std::endl;
		}

	}
	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}