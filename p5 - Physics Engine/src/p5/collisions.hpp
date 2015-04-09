#ifndef _462_COLLISIONS_HPP_
#define _462_COLLISIONS_HPP_

#include "scene/sphere.hpp"
#include "p5/spherebody.hpp"
#include "p5/trianglebody.hpp"
#include "p5/planebody.hpp"
#include "p5/modelbody.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping );
bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping );
bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping );
bool collides( SphereBody& body1, ModelBody& body2, real_t collision_damping );

// additional functions
real_t relative_velocity(SphereBody& body1, Vector3 velocity, Vector3 position);
Vector3 damping(Vector3 velocity, real_t collision_damping);
bool is_within_barycentric(const Vector3 point, const Vector3 vertices[3]);
bool is_hit_triangle(SphereBody& body1, const Vector3 vertices[3]);
real_t minimum_distance(Vector3 p, Vector3 v, Vector3 w);
}

#endif
