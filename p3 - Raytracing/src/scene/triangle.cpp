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
Intersection Triangle::hasHit(Ray& r) {
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

	// M = a(ei - hf) + b(gf - di) + c(dh - eg)
	double M = (a*((e*i) - (h*f))) + (b*((g*f) - (d*i))) + (c*((d*h) - (e*g)));

	// COMPUTE t
	// t = - (f(ak - jb) + e(jc - al) + d(bl - kc)) / M;
	real_t t = -(f*(a*k - j*b) + e*(j*c - a*l) + d*(b*l - k*c)) / M;
	if (t < intersection.epsilon || t > intersection.t) {
		return intersection;
	}

	// COMPUTE gamma
	// gamma = (i(ak - jb) + h(jc - al) + g(bl - kc)) / M;
	real_t gamma = (i*(a*k - j*b) + h*(j*c - a*l) + g*(b*l - k*c)) / M;
	if (gamma < 0 || gamma > 1) {
		return intersection;
	}

	// COMPUTE beta
	// beta = (j(ei - hf) + k(gf - di) + l(dh - eg)) / M;
	real_t beta = (j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g)) / M;
	if (beta < 0 || beta >(1 - gamma)) {
		return intersection;
	}

	intersection.t = t;
	return intersection;
}
} /* _462 */
