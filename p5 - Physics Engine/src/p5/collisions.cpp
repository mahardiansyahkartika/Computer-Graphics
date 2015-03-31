#include "p5/collisions.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	real_t distance = length(body2.position - body1.position);
	
	// collide
	if (distance <= body1.radius + body2.radius) {
		// calculate new velocity
		Vector3 direction = (body2.position - body1.position) / distance;

		Vector3 v1_new, v2_new;

		v1_new = body1.velocity - body2.velocity;
		v2_new = body2.velocity + (2 * direction * (body1.mass / (body1.mass + body2.mass)) * dot(v1_new, direction));
		v1_new = ((body1.mass * body1.velocity) + (body2.mass * body2.velocity) - (body2.mass * v2_new)) / body1.mass;

		body1.velocity = v1_new;
		body2.velocity = v2_new;

		return true;
	}

    return false;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	// get the triangle vertices
	Vector3 v_a = body2.vertices[0];
	Vector3 v_b = body2.vertices[1];
	Vector3 v_c = body2.vertices[2];

	// calculate normal
	Vector3 normal = normalize(cross(v_b - v_a, v_c - v_a));

	real_t distance = dot(body1.position - body2.position, normal);

	Vector3 hit_position = body2.position - distance * normal;
	
	if (abs(distance) <= body1.radius) {
		Vector3 v_new = body1.velocity - (2 * dot(body1.velocity, normal) * normal);
		
		body1.velocity = v_new;

		return true;
	}
    return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	real_t distance = dot(body1.position - body2.position, body2.normal);

	// collide
	if (abs(distance) <= body1.radius) {
		Vector3 v_new = body1.velocity - (2 * dot(body1.velocity, body2.normal) * body2.normal);

		body1.velocity = v_new;

		return true;
	}

	return false;
}

bool collides( SphereBody& body1, ModelBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity

    return false;
}

}
