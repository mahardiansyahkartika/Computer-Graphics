#include "p5/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
    position = sphere->position;
    radius = sphere->radius;
    orientation = sphere->orientation;
    mass = 0.0;
    velocity = Vector3::Zero();
    angular_velocity = Vector3::Zero();
    force = Vector3::Zero();
    torque = Vector3::Zero();
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in position dt in the future

	velocity += (force / mass) * dt;
	velocity *= 1.0 - motion_damping;

	Vector3 delta_position = velocity * dt;
	position += delta_position;

	// update graphical object
	sphere->position = position;

    return delta_position;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in orientation dt in the future
    // vec.x = rotation along x axis
    // vec.y = rotation along y axis
    // vec.z = rotation along z axis

	// moment of inertia
	real_t I = 0.4 * mass * radius * radius;
	angular_velocity += (torque / I) * dt;
	angular_velocity *= 1.0 - motion_damping;

	Vector3 delta_orientation = angular_velocity * dt;

	// pitch
	Vector3 axis_pitch = orientation * Vector3::UnitX();
	real_t radian_pitch = delta_orientation.x;
	orientation = normalize(Quaternion(axis_pitch, radian_pitch) * orientation);
	// yaw
	Vector3 axis_yaw = orientation * Vector3::UnitY();
	real_t radian_yaw = delta_orientation.y;
	orientation = normalize(Quaternion(axis_yaw, radian_yaw) * orientation);
	// roll
	Vector3 axis_roll = orientation * Vector3::UnitZ();
	real_t radian_roll = delta_orientation.z;
	orientation = normalize(Quaternion(axis_roll, radian_roll) * orientation);

	// update graphical object
	sphere->orientation = orientation;

    return delta_orientation;
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    // TODO apply force/torque to sphere

	// if offset is zero (less than epsilon), only linear component
	if (length(offset) < 1e-9) {
		// all force goes to linear
		force += f;
		// torque is zero
		torque += Vector3::Zero();
	} else { // if offset is not equal zero, both linear and angular components
		// linear term is the projection of force on the offset
		force += dot(f, offset) * offset / length(offset) / length(offset);
		// angular term is cross product between force and offset
		// NOTE: order matters
		torque += cross(offset, f);
	}
}

void SphereBody::clear_force() {
	force = Vector3::Zero();
	torque = Vector3::Zero();
}

}
