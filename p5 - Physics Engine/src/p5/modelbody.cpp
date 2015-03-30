#include "p5/modelbody.hpp"
#include <iostream>

namespace _462 {

ModelBody::ModelBody( Model* geom )
{
    model = geom;
}

Vector3 ModelBody::step_position( real_t dt, real_t motion_damping )
{
    return Vector3::Zero();
}

Vector3 ModelBody::step_orientation( real_t dt, real_t motion_damping )
{
    return Vector3::Zero();
}

void ModelBody::apply_force( const Vector3& f, const Vector3& offset )
{
    return;
}

}
