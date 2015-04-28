#include "scene/geometry.hpp"

namespace _462 {

Geometry::Geometry():
    position( Vector3::Zero() ),
    orientation( Quaternion::Identity() ),
    scale( Vector3::Ones() )
{

}

Geometry::~Geometry() { } 

bool Geometry::initialize()
{
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);
	
	return true;
}

}
