/**
 * @file raytacer.hpp
 * @brief Raytracer class
 *
 * Implement these functions for project 3.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_RAYTRACER_HPP_
#define _462_RAYTRACER_HPP_

#include "math/color.hpp"
#include "math/random462.hpp"
#include "p3/photon.hpp"
#include "p3/neighbor.hpp"
#include "application/opengl.hpp"
#include "p3/photonmap.hpp"
#include "p3/util.hpp"
#include <stack> 
#include "application/application.hpp"

namespace _462 {

class Scene;
class Ray;
struct Intersection;
class Raytracer
{
public:
    PhotonMap photonMap;

    Raytracer();

    ~Raytracer();

    bool initialize(Scene* scene, size_t num_samples,
                    size_t width, size_t height, Options opt);
	Color3 trace_ray(Ray &ray, const Scene* scene, int depth/*more args*/);
    
    bool raytrace(unsigned char* buffer, real_t* max_time);
    
    Color3 trace_pixel(size_t x,
               size_t y,
               size_t width,
               size_t height);

	// additional attributes
	// Depth of Field
	bool isDof = false;
	real_t dofFocalLength;
	real_t dofApertureSize;
	size_t dofTotalRay;

	// additional functions
	Intersection raycast(Ray& ray, const Scene* scene, real_t t1 = -1);
	Color3 shadowRays(const Scene* scene, const Intersection intersection);
	Color3 getReflectionColor(Ray& ray, const Scene* scene, const Intersection intersection, int depth, Vector3 normal);
	Color3 getRefractionColor(Ray& ray, const Scene* scene, const Intersection intersection, int depth, Vector3 normal, real_t ratio);
private:
    // the scene to trace
    Scene* scene;
    Projector projector;
    
    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;

    unsigned int num_samples;
    
};

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
