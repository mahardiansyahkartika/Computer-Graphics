/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "application/opengl.hpp"
#include <algorithm>

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define BUMP_FACTOR 1

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere() : radius(0), material(0) {	}

Sphere::~Sphere() {}

void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}

//quadratic formula
//If a solution exists, returns answers in x1 and x2, and returns true.
//Otherwise, returns false
bool solve_quadratic(real_t *x1,real_t *x2, real_t a, real_t b, real_t c){
    real_t b24ac = b*b-4*a*c;
    if(b24ac<0){
        return false;
    }else{
        real_t sb24ac=sqrt(b24ac);
        *x1=(-b+sb24ac)/(2*a);
        *x2=(-b-sb24ac)/(2*a);
        return true;
    }
}
//solve a quadratic equation, and then return the smallest solution larger than EPS
//if there is no solution, return -1
real_t solve_time(real_t a,real_t b,real_t c){
    real_t x1;
    real_t x2;
    if(solve_quadratic(&x1,&x2,a,b,c)){
        if(x1>EPS && x2>EPS){
            return std::min(x1,x2);
        }else if(x1>EPS){
            return x1;
        }else if(x2>EPS){
            return x2;
        }
    }
    return -1;
}

// additional functions
Intersection Sphere::getIntersection(Ray& r) {
	Intersection intersection;

	// inverse e & d point
	Vector4 iE = invMat * Vector4(r.e.x, r.e.y, r.e.z, 1);
	Vector4 iD = invMat * Vector4(r.d.x, r.d.y, r.d.z, 0);

	// create ray in the object's local space
	Vector3 e = Vector3(iE.x, iE.y, iE.z);
	Vector3 d = Vector3(iD.x, iD.y, iD.z);

	Vector3 c = Vector3(0, 0, 0);

	// check bounding box
	if (boundBox.intersects(Ray(e, d))) {
		// compute discriminant
		real_t A = dot(d, d);
		real_t B = 2 * dot(d, e - c);
		real_t C = dot(e - c, e - c) - pow(radius, 2);
		// discriminant = (d � (e - c))^2 - (d � d) ((e - c) � (e - c) - R^2);
		//real_t discriminant = pow(dot(d, e-c), 2) - (dot(d, d))*(dot(e-c, e-c) - pow(radius, 2));
		real_t discriminant = (B*B - (4 * A*C));

		// no solution
		if (discriminant < 0) {
			return intersection;
		}

		real_t t = solve_time(A, B, C);

		if (t > intersection.t || t < intersection.epsilon) {
			return intersection;
		}

		intersection.t = t;
		intersection.ray = r;
		intersection.localRay = Ray(e, d);
	}

	return intersection;
}

void Sphere::processHit(Intersection& hit)
{
	// compute position
	hit.int_point.position = hit.ray.e + (hit.ray.d*hit.t);

	Vector3 localNormal = hit.localRay.e + (hit.localRay.d*hit.t);
	hit.int_point.normal = normalize(normMat * normalize(localNormal));

	// compute texture coordinate on sphere
	hit.int_point.tex_coord = getTextureCoordinate(hit.int_point.position);

	// populate the material properties
	hit.int_material.ambient = material->ambient;
	hit.int_material.diffuse = material->diffuse;
	hit.int_material.specular = material->specular;
	hit.int_material.refractive_index = material->refractive_index;

	int width, height;
	int pix_x, pix_y;
	material->texture.get_texture_size(&width, &height);
	pix_x = (int)fmod(width*hit.int_point.tex_coord.x, width);
	pix_y = (int)fmod(height*hit.int_point.tex_coord.y, height);

	hit.int_material.texture = material->texture.get_texture_pixel(pix_x, pix_y);

	return;
}

Vector2 Sphere::getTextureCoordinate(Vector3 hitPosition)
{
	Vector2 tex_coord;
	real_t theta = acosf((hitPosition.y - position.y) / radius);
	real_t phi = atan2(hitPosition.x - position.x, hitPosition.z - position.z);
	// if phi is negative adding 2PI to phi
	if (phi < 0) phi = phi + (2 * PI);

	tex_coord.x = phi / (2 * PI);
	tex_coord.y = (PI - theta) / PI;
	return tex_coord;
}

Bound Sphere::createBoundingBox() {
	Vector3 center = Vector3(0, 0, 0);
	Vector3 min = center - Vector3(radius, radius, radius);
	Vector3 max = center + Vector3(radius, radius, radius);
	return Bound(min, max);
}
} /* _462 */

