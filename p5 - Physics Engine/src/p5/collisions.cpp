#include "p5/collisions.hpp"

namespace _462 {

bool collides(SphereBody& body1, SphereBody& body2, real_t collision_damping)
{
    // TODO detect collision. If there is one, update velocity

	// relative velocity of one object to the other is positive
	if (relative_velocity(body1, body2, body2.position) >= 0) {
		real_t distance = length(body2.position - body1.position);

		// collide
		if (distance <= body1.radius + body2.radius) {
			// calculate new velocity
			Vector3 direction = (body2.position - body1.position) / distance;

			Vector3 v1_new, v2_new;

			v1_new = body1.velocity - body2.velocity;
			v2_new = body2.velocity + (2 * direction * (body1.mass / (body1.mass + body2.mass)) * dot(v1_new, direction));
			v1_new = ((body1.mass * body1.velocity) + (body2.mass * body2.velocity) - (body2.mass * v2_new)) / body1.mass;

			// update velocity
			body1.velocity = damping(v1_new, collision_damping);
			body2.velocity = damping(v2_new, collision_damping);

			return true;
		}
	}
    return false;
}

bool collides(SphereBody& body1, TriangleBody& body2, real_t collision_damping)
{
	// TODO detect collision. If there is one, update velocity

	// get the triangle vertices
	Vector3 v_a = body2.vertices[0];
	Vector3 v_b = body2.vertices[1];
	Vector3 v_c = body2.vertices[2];

	// calculate normal
	Vector3 normal = normalize(cross(v_b - v_a, v_c - v_a));

	real_t distance = dot(body1.position - body2.position, normal);
	Vector3 hit_point = body1.position - distance * normal;

	// relative velocity of one object to the other is positive
	if (relative_velocity(body1, body2, hit_point) >= 0) {
		// check distance and check whether the hit_point within barycentric coordinates
		real_t beta, gamma;
		if (abs(distance) <= body1.radius && is_within_barycentric(hit_point, body2.vertices, beta, gamma)) {
			Vector3 v_new = body1.velocity - (real_t(2) * dot(body1.velocity, normal) * normal);

			// update velocity
			body1.velocity = damping(v_new, collision_damping);

			return true;
		}
	}
    return false;
}

bool collides(SphereBody& body1, PlaneBody& body2, real_t collision_damping)
{
    // TODO detect collision. If there is one, update velocity

	real_t distance = dot(body1.position - body2.position, body2.normal);
	Vector3 hit_point = body1.position - distance * body2.normal;

	// relative velocity of one object to the other is positive
	if (relative_velocity(body1, body2, hit_point) >= 0) {
		// collide
		if (abs(distance) <= body1.radius) {
			Vector3 v_new = body1.velocity - (2 * dot(body1.velocity, body2.normal) * body2.normal);
			
			// update velocity
			body1.velocity = damping(v_new, collision_damping);

			return true;
		}
		return false;
	}
}

bool collides(SphereBody& body1, ModelBody& body2, real_t collision_damping)
{
    // TODO detect collision. If there is one, update velocity

	// iterate through all the triangles in the mesh
	bool is_collide = false;

	const Mesh* mesh = body2.model->mesh;
		
	for (size_t i = 0; i < mesh->num_triangles(); ++i) {
		// get the triangle vertices
		MeshTriangle idxTriangle = mesh->get_triangles()[i];
		const MeshVertex* vertices = mesh->get_vertices();

		MeshVertex v_a = vertices[idxTriangle.vertices[0]];
		MeshVertex v_b = vertices[idxTriangle.vertices[1]];
		MeshVertex v_c = vertices[idxTriangle.vertices[2]];

		// calculate normal
		Vector3 normal_plane = normalize(cross(v_b.position - v_a.position, v_c.position - v_a.position));

		real_t distance = dot(body1.position - body2.position, normal_plane);
		Vector3 hit_point = body1.position - distance * normal_plane;

		// relative velocity of one object to the other is positive
		if (relative_velocity(body1, body2, hit_point) >= 0) {
			Vector3 vertices_pos[3] { v_a.position, v_b.position, v_c.position };
			
			// check distance and check whether the hit_point within barycentric coordinates
			real_t beta, gamma;
			if (abs(distance) <= body1.radius && is_within_barycentric(hit_point, vertices_pos, beta, gamma)) {
				// calculate normal
				Vector3 normal = ((real_t(1) - beta - gamma)*v_a.normal) + (beta*v_b.normal) + (gamma*v_c.normal);

				Vector3 v_new = body1.velocity - (real_t(2) * dot(body1.velocity, normal) * normal);

				// update velocity
				body1.velocity = damping(v_new, collision_damping);

				is_collide = true;
			}
		}
	}

	return is_collide;
}

real_t relative_velocity(SphereBody& body1, Body& body2, Vector3 position) {
	Vector3 r_v = body1.velocity - body2.velocity;
	return dot(normalize(r_v), normalize(position - body1.position));
}

Vector3 damping(Vector3 velocity, real_t collision_damping) {
	// check epsilon, so that we don't keep on making minor changes
	if (length(velocity) <= 0.01f)
		return Vector3::Zero();

	return velocity - collision_damping * velocity;
}

bool is_within_barycentric(const Vector3 point, const Vector3 vertices[3], real_t& beta, real_t& gamma) {
	// compute vector
	Vector3 v0 = vertices[2] - vertices[0]; // C - A
	Vector3 v1 = vertices[1] - vertices[0]; // B - A
	Vector3 v2 = point - vertices[0]; // P - A

	// compute dot products
	real_t dot00 = dot(v0, v0);
	real_t dot01 = dot(v0, v1);
	real_t dot02 = dot(v0, v2);
	real_t dot11 = dot(v1, v1);
	real_t dot12 = dot(v1, v2);

	// Compute barycentric coordinates
	real_t invDenom = real_t(1) / (dot00 * dot11 - dot01 * dot01);
	beta = (dot00 * dot12 - dot01 * dot02) * invDenom;
	gamma = (dot11 * dot02 - dot01 * dot12) * invDenom;

	// Check if point is in triangle
	return (beta >= 0) && (gamma >= 0) && (beta + gamma < 1);
}
}
