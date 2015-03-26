/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"
#include "math/math.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
    isBig=true;
}

Triangle::~Triangle() { }

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

#if REAL_FLOAT
    glNormal3fv( &vertices[0].normal.x );
    glTexCoord2fv( &vertices[0].tex_coord.x );
    glVertex3fv( &vertices[0].position.x );

    glNormal3fv( &vertices[1].normal.x );
    glTexCoord2fv( &vertices[1].tex_coord.x );
    glVertex3fv( &vertices[1].position.x);

    glNormal3fv( &vertices[2].normal.x );
    glTexCoord2fv( &vertices[2].tex_coord.x );
    glVertex3fv( &vertices[2].position.x);
#else
    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);
#endif

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

// additional functions
Intersection Triangle::getIntersection(Ray& r) {
	Intersection intersection;

	// inverse e & d point
	Vector4 iE = invMat * Vector4(r.e.x, r.e.y, r.e.z, 1);
	Vector4 iD = invMat * Vector4(r.d.x, r.d.y, r.d.z, 0);

	// create ray in the object's local space
	Ray ray(Vector3(iE.x, iE.y, iE.z), Vector3(iD.x, iD.y, iD.z));

	// all vertices
	Vertex vA = vertices[0];
	Vertex vB = vertices[1];
	Vertex vC = vertices[2];

	// Cramer's Rule
	// | Xa-Xb  Xa-Xc  Xd | | beta  |   | Xa - Xe |      | a  d  g | | beta  |   | j |
	// | Ya-Yb  Ya-Yc  Yd | | gamma | = | Ya - Ye | ===> | b  e  h | | gamma | = | k |
	// | Za-Zb  Xa-Xc  Zd | |   t   |   | Za - Ze |      | c  f  i | |   t   |   | l |
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
	if (t < intersection.epsilon || t > intersection.t) {
		return intersection;
	}

	// COMPUTE gamma
	// gamma = (i(ak - jb) + h(jc - al) + g(bl - kc)) / M;
	real_t gamma = (i*ak_minus_jb + h*jc_minus_al + g*bl_minus_kc) / M;
	if (gamma < 0 || gamma > 1) {
		return intersection;
	}

	// COMPUTE beta
	// beta = (j(ei - hf) + k(gf - di) + l(dh - eg)) / M;
	real_t beta = (j*ei_minus_hf + k*gf_minus_di + l*dh_minus_eg) / M;
	if (beta < 0 || beta >(1 - gamma)) {
		return intersection;
	}

	// update intersection
	intersection.t = t;
	intersection.beta = beta;
	intersection.gamma = gamma;
	intersection.ray = r; // save the untransformed ray

	return intersection;
}

void Triangle::processHit(Intersection& hit) {
	/// cast the intersection object in the local type
	Intersection thisHit = static_cast<Intersection>(hit);
	/// compute alpha
	real_t alpha = 1.0 - (thisHit.beta + thisHit.gamma);

	/// get the triangle vertices
	Vertex v_a = vertices[0];
	Vertex v_b = vertices[1];
	Vertex v_c = vertices[2];

	/// store the hit coordinate in parent structure
	hit.int_point.position = hit.ray.e + (hit.ray.d*hit.t);

	v_a.normal = normalize(v_a.normal);
	v_b.normal = normalize(v_b.normal);
	v_c.normal = normalize(v_c.normal);

	//Vector3 localNormal = (alpha*v_a.normal) + (thisHit.beta*v_b.normal) + (thisHit.gamma*v_c.normal);
	Vector3 localNormal = cross(v_b.position - v_a.position, v_c.position - v_a.position);
	hit.int_point.normal = normalize(normMat * localNormal);

	hit.int_point.tex_coord = (alpha*v_a.tex_coord) + (thisHit.beta*v_b.tex_coord) + (thisHit.gamma*v_c.tex_coord);

	interpolateMaterials(hit, alpha, thisHit.beta, thisHit.gamma);

	return;
}

void Triangle::interpolateMaterials(Intersection& hit, real_t alpha, real_t beta, real_t gamma) {
	/// Get the triangle vertices
	Vertex v_a = vertices[0];
	Vertex v_b = vertices[1];
	Vertex v_c = vertices[2];

	/// interpolate ambient color
	hit.int_material.ambient = (alpha * v_a.material->ambient) + (beta * v_b.material->ambient) + (gamma * v_c.material->ambient);
	/// interpolate diffuse color
	hit.int_material.diffuse = (alpha * v_a.material->diffuse) + (beta * v_b.material->diffuse) + (gamma * v_c.material->diffuse);
	/// interpolate specular color
	hit.int_material.specular = (alpha * v_a.material->specular) + (beta * v_b.material->specular) + (gamma * v_c.material->specular);
	/// interpolate refractive index
	hit.int_material.refractive_index = (alpha * v_a.material->refractive_index) + (beta * v_b.material->refractive_index) + (gamma * v_c.material->refractive_index);

	int width, height;
	int pix_x, pix_y;

	/// texture color for vertex a
	v_a.material->texture.get_texture_size(&width, &height);
	pix_x = (int)fmod(width*hit.int_point.tex_coord.x, width);
	pix_y = (int)fmod(height*hit.int_point.tex_coord.y, height);
	Color3 a_tex = v_a.material->texture.get_texture_pixel(pix_x, pix_y);

	/// texture color for vertex b
	v_b.material->texture.get_texture_size(&width, &height);
	pix_x = (int)fmod(width*hit.int_point.tex_coord.x, width);
	pix_y = (int)fmod(height*hit.int_point.tex_coord.y, height);
	Color3 b_tex = v_b.material->texture.get_texture_pixel(pix_x, pix_y);

	/// texture color for vertex c
	v_c.material->texture.get_texture_size(&width, &height);
	pix_x = (int)fmod(width*hit.int_point.tex_coord.x, width);
	pix_y = (int)fmod(height*hit.int_point.tex_coord.y, height);
	Color3 c_tex = v_c.material->texture.get_texture_pixel(pix_x, pix_y);

	/// interpolate the texture colors
	hit.int_material.texture = (alpha * a_tex) + (beta * b_tex) + (gamma * c_tex);
}
} /* _462 */
