/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
}

Triangle::~Triangle() { }

bool Triangle::initialize(){
	Geometry::initialize();

	// create bounding box
	boundBox = createBoundingBox();

	return true;
}

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

Bound Triangle::createBoundingBox() {
	Vector3 min = vertices[0].position;
	Vector3 max = vertices[0].position;

	for (int i = 1; i < 3; ++i) {
		if (vertices[i].position.x < min.x) min.x = vertices[i].position.x;
		if (vertices[i].position.y < min.y) min.y = vertices[i].position.y;
		if (vertices[i].position.z < min.z) min.z = vertices[i].position.z;
		if (vertices[i].position.x > max.x) max.x = vertices[i].position.x;
		if (vertices[i].position.y > max.y) max.y = vertices[i].position.y;
		if (vertices[i].position.z > max.z) max.z = vertices[i].position.z;
	}

	return Bound(min, max);
}

} /* _462 */

