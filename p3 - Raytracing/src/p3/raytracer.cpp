/**
* @file raytacer.cpp
* @brief Raytracer class
*
* Implement these functions for project 4.
*
* @author H. Q. Bovik (hqbovik)
* @bug Unimplemented
*/

#include "raytracer.hpp"
#include "scene/scene.hpp"
#include "math/quickselect.hpp"
#include "p3/randomgeo.hpp"
#include <SDL_timer.h>
namespace _462 {

//number of rows to render before updating the result
static const unsigned STEP_SIZE = 1;
static const unsigned CHUNK_SIZE = 1;

#define MONTE_CARLO_SAMPLES 1
#define RECURSIVE_LIGHT 3

Raytracer::Raytracer() {
	scene = 0;
	width = 0;
	height = 0;
}

Raytracer::~Raytracer() { }

/**
* Initializes the raytracer for the given scene. Overrides any previous
* initializations. May be invoked before a previous raytrace completes.
* @param scene The scene to raytrace.
* @param width The width of the image being raytraced.
* @param height The height of the image being raytraced.
* @return true on success, false on error. The raytrace will abort if
*  false is returned.
*/
bool Raytracer::initialize(Scene* scene, size_t num_samples,
	size_t width, size_t height)
{
	this->scene = scene;
	this->num_samples = num_samples;
	this->width = width;
	this->height = height;

	current_row = 0;

	projector.init(scene->camera);
	scene->initialize();
	photonMap.initialize(scene);
	return true;
}
//compute ambient lighting
Color3 Raytracer::trace_ray(Ray &ray, const Scene* scene, int depth/*maybe some more arguments*/){
	//TODO: render something more interesting
	//return Color3(fabs(sin(10 * ray.d.x)), fabs(10 * cos(ray.d.y)), fabs(10 * tan(ray.d.y)));

	// check if ray hits an object in the scene
	Intersection intersection = raycast(ray, scene);

	/// declare all the recursion variables
	Vector3 intersectionNormal;
	Vector3 incomingDirection;
	real_t n, n_t;

	if (intersection.t == std::numeric_limits<real_t>::infinity()) {
		return scene->background_color;
	}
	else { // hit object
		/// end of recursion
		if (depth-- <= 0) {
			return Color3(0, 0, 0);
		}

		// REFLECTION
		if (intersection.int_material.refractive_index == 0) { // opaque object
			// DIFFUSE & AMBIENT + REFLECTION
			return shadowRays(scene, intersection) + getReflectionColor(ray, scene, intersection, depth, intersection.int_point.normal);
		}
		// REFRACTION
		else {
			intersectionNormal = intersection.int_point.normal;
			incomingDirection = normalize(ray.d);

			bool isOutsideGeometry = false;
			if (dot(incomingDirection, intersectionNormal) < 0) { // entering dielectric
				n = scene->refractive_index;
				n_t = intersection.int_material.refractive_index;

				isOutsideGeometry = true;
			}
			else { // exiting dielectric
				n = intersection.int_material.refractive_index;
				n_t = scene->refractive_index;
			}

			Vector3 normal = intersectionNormal;

			real_t R = computeFresnelCoefficient(intersection, ray, n, n_t);
			if (isOutsideGeometry) { // check fresnell
				// Probability: reflection = R, refraction = 1-R
				return (R * getReflectionColor(ray, scene, intersection, depth, normal)) + ((1 - R) * getRefractionColor(ray, scene, intersection, depth, normal, n / n_t));
			}
			else { // inverse normal
				normal *= -1;
			}

			return getRefractionColor(ray, scene, intersection, depth, normal, n / n_t);
		}
	}
}

Color3 Raytracer::getRefractionColor(Ray& ray, const Scene* scene, const Intersection intersection, int depth, Vector3 normal, real_t ratio) {
	// create a refractive ray
	Vector3 refractiveVector = refract(normal, normalize(ray.d), ratio);
	if (refractiveVector == Vector3(0, 0, 0)){
		// Total Internal Reflection
		return shadowRays(scene, intersection) + getReflectionColor(ray, scene, intersection, depth, normal);
	}

	Ray refractedRay = Ray(intersection.int_point.position, refractiveVector);
	return trace_ray(refractedRay, scene, depth);
}

Color3 Raytracer::getReflectionColor(Ray& ray, const Scene* scene, const Intersection intersection, int depth, Vector3 normal) {
	// create a reflected ray
	Vector3 reflectionVector = reflect(normal, normalize(ray.d));
	Ray reflectedRay = Ray(intersection.int_point.position, reflectionVector);

	Color3 reflectionColor = trace_ray(reflectedRay, scene, depth);
	// multiply the returned color by the material's specular color and also by the texture color
	return reflectionColor * intersection.int_material.specular * intersection.int_material.texture;
}

/**
* Performs a raytrace on the given pixel on the current scene.
* The pixel is relative to the bottom-left corner of the image.
* @param scene The scene to trace.
* @param x The x-coordinate of the pixel to trace.
* @param y The y-coordinate of the pixel to trace.
* @param width The width of the screen in pixels.
* @param height The height of the screen in pixels.
* @return The color of that pixel in the final image.
*/
Color3 Raytracer::trace_pixel(size_t x,
	size_t y,
	size_t width,
	size_t height)
{
	assert(x < width);
	assert(y < height);

	real_t dx = real_t(1) / width;
	real_t dy = real_t(1) / height;

	Color3 res = Color3::Black();

	unsigned int iter;
	for (iter = 0; iter < num_samples; iter++)
	{
		// pick a point within the pixel boundaries to fire our
		// ray through.
		real_t i = real_t(2)*(real_t(x) + random_uniform())*dx - real_t(1);
		real_t j = real_t(2)*(real_t(y) + random_uniform())*dy - real_t(1);

		Ray r = Ray(scene->camera.get_position(), projector.get_pixel_dir(i, j));

		res += trace_ray(r, scene, RECURSIVE_LIGHT);
		// TODO return the color of the given pixel
		// you don't have to use this stub function if you prefer to
		// write your own version of Raytracer::raytrace.

	}
	return res*(real_t(1) / num_samples);
}

/**
* Raytraces some portion of the scene. Should raytrace for about
* max_time duration and then return, even if the raytrace is not copmlete.
* The results should be placed in the given buffer.
* @param buffer The buffer into which to place the color data. It is
*  32-bit RGBA (4 bytes per pixel), in row-major order.
* @param max_time, If non-null, the maximum suggested time this
*  function raytrace before returning, in seconds. If null, the raytrace
*  should run to completion.
* @return true if the raytrace is complete, false if there is more
*  work to be done.
*/
bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time)
{

	static const size_t PRINT_INTERVAL = 64;

	// the time in milliseconds that we should stop
	unsigned int end_time = 0;
	bool is_done;

	if (max_time)
	{
		// convert duration to milliseconds
		unsigned int duration = (unsigned int)(*max_time * 1000);
		end_time = SDL_GetTicks() + duration;
	}

	// until time is up, run the raytrace. we render an entire group of
	// rows at once for simplicity and efficiency.
	for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE)
	{
		// we're done if we finish the last row
		is_done = current_row >= height;
		// break if we finish
		if (is_done) break;

		int loop_upper = std::min(current_row + STEP_SIZE, height);

		for (int c_row = current_row; c_row < loop_upper; c_row++)
		{
			/*
			* This defines a critical region of code that should be
			* executed sequentially.
			*/
#pragma omp critical
		{
			if (c_row % PRINT_INTERVAL == 0)
				printf("Raytracing (Row %d)\n", c_row);
		}

		// This tells OpenMP that this loop can be parallelized.
#pragma omp parallel for schedule(dynamic, CHUNK_SIZE)
		for (int x = 0; x < width; x++)
		{
			// trace a pixel
			Color3 color = trace_pixel(x, c_row, width, height);
			// write the result to the buffer, always use 1.0 as the alpha
			color.to_array4(&buffer[4 * (c_row * width + x)]);
		}
#pragma omp barrier

		}
	}

	if (is_done) printf("Done raytracing!\n");

	return is_done;
}

// ---------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------- ADDITIONAL FUNCTIONS --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------- //
Intersection Raytracer::raycast(Ray& ray, const Scene* scene, real_t t1) {
	// get scene geometries
	Geometry* const* sceneGeometries = scene->get_geometries();

	Intersection *closestIntersection = &Intersection();

	/// check if the ray needs to be a certain length
	if (t1 > 0)
		(*closestIntersection).t = t1;

	// check all for all objects in the scene
	for (unsigned int i = 0; i < scene->num_geometries(); ++i)
	{
		Intersection currIntersection = sceneGeometries[i]->getIntersection(ray);
		// check if this geometry is closer to ray
		if (currIntersection.t < (*closestIntersection).t) {
			closestIntersection = &sceneGeometries[i]->getIntersection(ray);
			(*closestIntersection).index = i;
		}
	}

	// if hit, process hit
	if ((*closestIntersection).index >= 0)
		sceneGeometries[(*closestIntersection).index]->processHit(*closestIntersection);

	return *closestIntersection;
}

Color3 Raytracer::shadowRays(const Scene* scene, const Intersection intersection) {
	// instantitate required diffuse and ambient material/ light colors
	Color3 k_d = intersection.int_material.diffuse;
	Color3 k_a = intersection.int_material.ambient;
	Color3 c_a = scene->ambient_light;
	// get texture color
	Color3 t_p = intersection.int_material.texture;

	// instantiate the normal vector for the intersection
	Vector3 normal = intersection.int_point.normal;
	/// variable to sum over all light sources
	Color3 avgLightColor(0.0, 0.0, 0.0);

	int numSamples = MONTE_CARLO_SAMPLES; // Monte Carlo

	// iterate through the light sources
	for (unsigned int i = 0; i < scene->num_lights(); ++i) {
		SphereLight currentLight = scene->get_lights()[i];

		// store color of light
		Color3 lightColor = currentLight.color;
		// store attenuation terms
		real_t a_c = currentLight.attenuation.constant;
		real_t a_l = currentLight.attenuation.linear;
		real_t a_q = currentLight.attenuation.quadratic;

		// instantiate a color accumulator to add light color contributions
		Color3 lightAccumulator(0.0, 0.0, 0.0);

		// monte carlo sampling = numSamples samples / light
		unsigned totalLoop = numSamples;

		while (totalLoop--) {
			Vector3 lightSample;
			lightSample.x = random_gaussian();
			lightSample.y = random_gaussian();
			lightSample.z = random_gaussian();

			// normalize the vector
			lightSample = normalize(lightSample);

			// scale the light sample
			lightSample = (currentLight.radius * lightSample);
			//transform the light sample
			lightSample += currentLight.position;

			Vector3 sampleDirection = normalize(lightSample - intersection.int_point.position);

			// instantiate light ray
			Ray L = Ray(intersection.int_point.position, sampleDirection);

			// compute t till the light intersection
			real_t t_light = length(lightSample - intersection.int_point.position);

			// check for obstruction in path to light
			Intersection lightIntersection = raycast(L, scene, t_light);

			// if the pointer points to a valid intersection container
			if (lightIntersection.index >= 0)
				continue;

			// light is not blocked by any geomtery
			// compute distance to light source
			real_t d = t_light;

			Color3 c_i = lightColor * (real_t(1.0) / (a_c + (d*a_l) + (pow(d, 2.0)*a_q)));

			real_t normalDotLight = dot(normal, L.d);
			if (normalDotLight > 0) {
				// sum over all samples sent to a light source
				lightAccumulator += (c_i * k_d * normalDotLight);
			}
		}
		// sum over all lights in scene
		avgLightColor += lightAccumulator*(real_t(1.0) / real_t(numSamples));
	}

	return t_p*((c_a*k_a) + avgLightColor);
}
} /* _462 */
