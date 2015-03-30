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
Model::~Model() {
	delete tree;
}

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

	// create tree
	std::cout << "Start creating Tree" << std::endl;
	tree = new MeshTree(mesh);
	std::cout << "Creating tree done" << std::endl;

    return true;
}

// additional functions
void Model::hit(Ray& r, MeshTreeNode* node, Intersection* intersection) {
	if (node == NULL)
		return;

	if (node->bbox->intersects(r)) {
		// if still has child
		if (node->left != NULL || node->right != NULL) {
			hit(r, node->left, intersection);
			hit(r, node->right, intersection);
		}
		else { // we have reached leaf
			// iterate all triangles in node
			for (size_t idxTri = 0; idxTri < node->triangles.size(); ++idxTri) {
				// all vertices
				MeshVertex vA = mesh->vertices[node->triangles[idxTri].vertices[0]];
				MeshVertex vB = mesh->vertices[node->triangles[idxTri].vertices[1]];
				MeshVertex vC = mesh->vertices[node->triangles[idxTri].vertices[2]];

				// Cramer's Rule
				double a = vA.position.x - vB.position.x;
				double b = vA.position.y - vB.position.y;
				double c = vA.position.z - vB.position.z;
				double d = vA.position.x - vC.position.x;
				double e = vA.position.y - vC.position.y;
				double f = vA.position.z - vC.position.z;
				double g = r.d.x;
				double h = r.d.y;
				double i = r.d.z;
				double j = vA.position.x - r.e.x;
				double k = vA.position.y - r.e.y;
				double l = vA.position.z - r.e.z;

				// reduce number of operation
				double ei_minus_hf = (e*i) - (h*f);
				double gf_minus_di = (g*f) - (d*i);
				double dh_minus_eg = (d*h) - (e*g);
				double ak_minus_jb = (a*k) - (j*b);
				double jc_minus_al = (j*c) - (a*l);
				double bl_minus_kc = (b*l) - (k*c);

				// M = a(ei - hf) + b(gf - di) + c(dh - eg)
				double M = a*ei_minus_hf + b*gf_minus_di + c*dh_minus_eg;

				// COMPUTE t
				// t = - (f(ak - jb) + e(jc - al) + d(bl - kc)) / M;
				real_t t = -(f*ak_minus_jb + e*jc_minus_al + d*bl_minus_kc) / M;
				if (t < intersection->epsilon || t > intersection->t) {
					continue;
				}

				// COMPUTE gamma
				// gamma = (i(ak - jb) + h(jc - al) + g(bl - kc)) / M;
				real_t gamma = (i*ak_minus_jb + h*jc_minus_al + g*bl_minus_kc) / M;
				if (gamma < 0 || gamma > 1) {
					continue;
				}

				// COMPUTE beta
				// beta = (j(ei - hf) + k(gf - di) + l(dh - eg)) / M;
				real_t beta = (j*ei_minus_hf + k*gf_minus_di + l*dh_minus_eg) / M;
				if (beta < 0 || beta >(1 - gamma)) {
					continue;
				}

				// update intersection
				intersection->t = t;
				intersection->beta = beta;
				intersection->gamma = gamma;
				intersection->hitTriangle = node->triangles[idxTri];
			}
		}
	}
}

Intersection* Model::getIntersection(Ray& r) {
	Intersection* closestIntersection = new Intersection();

	// inverse e & d point
	Vector4 iE = invMat * Vector4(r.e.x, r.e.y, r.e.z, 1);
	Vector4 iD = invMat * Vector4(r.d.x, r.d.y, r.d.z, 0);

	// create ray in the object's local space
	Ray ray(Vector3(iE.x, iE.y, iE.z), Vector3(iD.x, iD.y, iD.z));

	// check hit
	hit(ray, tree->root, closestIntersection);

	closestIntersection->ray = r;
	closestIntersection->localRay = ray;

	return closestIntersection;
}

void Model::processHit(Intersection* hit) {
	// compute alpha
	real_t alpha = 1.0 - (hit->beta + hit->gamma);

	MeshTriangle tri = hit->hitTriangle;

	MeshVertex v_a = mesh->vertices[tri.vertices[0]];
	MeshVertex v_b = mesh->vertices[tri.vertices[1]];
	MeshVertex v_c = mesh->vertices[tri.vertices[2]];

	hit->int_point.position = hit->ray.e + (hit->ray.d*hit->t);

	Vector3 localNormal = (alpha*v_a.normal) + (hit->beta*v_b.normal) + (hit->gamma*v_c.normal);
	hit->int_point.normal = normalize(normMat * normalize(localNormal));

	/// compute the texture coordinate
	hit->int_point.tex_coord = (alpha*v_a.tex_coord) + (hit->beta*v_b.tex_coord) + (hit->gamma*v_c.tex_coord);

	/// store the material details
	hit->int_material.diffuse = material->diffuse;
	hit->int_material.ambient = material->ambient;
	hit->int_material.specular = material->specular;
	hit->int_material.refractive_index = material->refractive_index;

	int width, height;
	int pix_x, pix_y;
	material->texture.get_texture_size(&width, &height);
	pix_x = (int)fmod(width*hit->int_point.tex_coord.x, width);
	pix_y = (int)fmod(height*hit->int_point.tex_coord.y, height);

	hit->int_material.texture = material->texture.get_texture_pixel(pix_x, pix_y);

	return;
}

Bound Model::createBoundingBox() {
	Vector3 min = mesh->vertices[mesh->triangles[0].vertices[0]].position;
	Vector3 max = mesh->vertices[mesh->triangles[0].vertices[0]].position;
	
	// iterate all triangles
	for (unsigned int idxTri = 0; idxTri < mesh->num_triangles(); ++idxTri) {
		MeshTriangle triangle = mesh->triangles[idxTri];
		for (int i = 0; i < 3; ++i) {
			MeshVertex vertices = mesh->vertices[triangle.vertices[i]];
			if (vertices.position.x < min.x) min.x = vertices.position.x;
			if (vertices.position.y < min.y) min.y = vertices.position.y;
			if (vertices.position.z < min.z) min.z = vertices.position.z;
			if (vertices.position.x > max.x) max.x = vertices.position.x;
			if (vertices.position.y > max.y) max.y = vertices.position.y;
			if (vertices.position.z > max.z) max.z = vertices.position.z;
		}
	}
	return Bound(min, max);
}
void Model::update(real_t delta_time) {}
} /* _462 */
