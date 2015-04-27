#ifndef _462_PHYSICS_SPHEREBODY_HPP_
#define _462_PHYSICS_SPHEREBODY_HPP_

#include "scene/sphere.hpp"
#include "p5/body.hpp"

namespace _462 {

struct Derivative
{
	float dx;
	float dv;
};

class Sphere;

class SphereBody : public Body
{
public:
    Sphere* sphere;
    real_t radius;
    real_t mass;
    Vector3 force;
    Vector3 torque;
	Derivative derivative;

    SphereBody( Sphere* geom );
    virtual ~SphereBody() { }
    virtual Vector3 step_position( real_t dt, real_t motion_damping );
    virtual Vector3 step_orientation( real_t dt, real_t motion_damping );
    virtual void apply_force( const Vector3& f, const Vector3& offset );
	virtual void clear_force();
};

}

#endif

