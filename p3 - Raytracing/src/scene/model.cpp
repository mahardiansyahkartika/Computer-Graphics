/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ) { }
Model::~Model() { }

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}
bool Model::initialize(){
    Geometry::initialize();
    return true;
}

// additional functions
Intersection Model::hasHit(Ray& r) {
	// inverse e & d point
	Vector4 iE = invMat * Vector4(r.e.x, r.e.y, r.e.z, 1);
	Vector4 iD = invMat * Vector4(r.d.x, r.d.y, r.d.z, 0);

	// create ray in the object's local space
	Ray ray(Vector3(iE.x, iE.y, iE.z), Vector3(iD.x, iD.y, iD.z));

	Intersection closestIntersection;

	// iterate all triangles
	for (unsigned int idxTri = 0; idxTri < mesh->num_triangles(); ++idxTri) {
		MeshTriangle triangle = mesh->triangles[idxTri];
		// all vertices
		MeshVertex vA = mesh->vertices[triangle.vertices[0]];
		MeshVertex vB = mesh->vertices[triangle.vertices[1]];
		MeshVertex vC = mesh->vertices[triangle.vertices[2]];

		// Cramer's Rule
		double a = vA.position.x - vB.position.x;
		double b = vA.position.y - vB.position.y;
		double c = vA.position.z - vB.position.z;
		double d = vA.position.x - vC.position.x;
		double e = vA.position.y - vC.position.y;
		double f = vA.position.z - vC.position.z;
		double g = ray.d.x;
		double h = ray.d.y;
		double i = ray.d.z;
		double j = vA.position.x - ray.e.x;
		double k = vA.position.y - ray.e.y;
		double l = vA.position.z - ray.e.z;

		// M = a(ei - hf) + b(gf - di) + c(dh - eg)
		double M = (a*((e*i) - (h*f))) + (b*((g*f) - (d*i))) + (c*((d*h) - (e*g)));

		// COMPUTE t
		// t = - (f(ak - jb) + e(jc - al) + d(bl - kc)) / M;
		real_t t = -(f*(a*k - j*b) + e*(j*c - a*l) + d*(b*l - k*c)) / M;
		if (t < closestIntersection.epsilon || t > closestIntersection.t) {
			continue;
		}

		// COMPUTE gamma
		// gamma = (i(ak - jb) + h(jc - al) + g(bl - kc)) / M;
		real_t gamma = (i*(a*k - j*b) + h*(j*c - a*l) + g*(b*l - k*c)) / M;
		if (gamma < 0 || gamma > 1) {
			continue;
		}

		// COMPUTE beta
		// beta = (j(ei - hf) + k(gf - di) + l(dh - eg)) / M;
		real_t beta = (j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g)) / M;
		if (beta < 0 || beta >(1 - gamma)) {
			continue;
		}

		// update intersection
		closestIntersection.t = t;
	}

	return closestIntersection;
}
} /* _462 */
