#include "p5/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

void Physics::step( real_t dt )
{
    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it

	// CHECK COLLISION
	check_collision();

	// UPDATE STEP
	update_step(dt);
}

void Physics::check_collision() {
	for (size_t i = 0; i < num_spheres(); ++i) {
		// check order: sphere, triangle, model, plane
		// spheres
		for (size_t j = i + 1; j < num_spheres(); ++j) {
			if (collides(spheres[i], spheres[j], collision_damping)) break;
		}
		// triangles
		for (size_t j = 0; j < num_triangles(); ++j) {
			if (collides(spheres[i], triangles[j], collision_damping)) break;
		}
		// models
		for (size_t j = 0; j < num_models(); ++j) {
			if (collides(spheres[i], models[j], collision_damping)) break;
		}
		// planes
		for (size_t j = 0; j < num_planes(); ++j) {
			if (collides(spheres[i], planes[j], collision_damping)) break;
		}
	}
}

void Physics::update_step(real_t dt) {
	// spheres
	for (size_t i = 0; i < num_spheres(); ++i) {
		spheres[i]->step_position(dt, collision_damping);
	}
	// triangles
	for (size_t i = 0; i < num_triangles(); ++i) {
		triangles[i]->step_position(dt, collision_damping);
	}
	// models
	for (size_t i = 0; i < num_models(); ++i) {
		models[i]->step_position(dt, collision_damping);
	}
	// planes
	for (size_t i = 0; i < num_planes(); ++i) {
		planes[i]->step_position(dt, collision_damping);
	}
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_model( ModelBody* m )
{
    models.push_back( m );
}

size_t Physics::num_models() const
{
    return models.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( ModelList::iterator i = models.begin(); i != models.end(); i++) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    models.clear();

    gravity = Vector3::Zero();
	collision_damping = 0.0;
}

}
