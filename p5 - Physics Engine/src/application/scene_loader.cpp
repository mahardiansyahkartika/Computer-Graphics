/**
 * @file scene_loader.cpp
 * @brief Scene Loader
 *
 * @author Eric Butler (edbutler)
 */

#include "application/scene_loader.hpp"

#include "scene/scene.hpp"
#include "scene/sphere.hpp"
#include "scene/model.hpp"
#include "scene/triangle.hpp"
#include "p5/physics.hpp"
#include "tinyxml/tinyxml.h"

#include <iostream>
#include <map>
#include <cstring>
#include <exception>

namespace _462 {

struct StrCompare
{
    bool operator() ( const char* s1, const char* s2 ) const
    {
        return strcmp( s1, s2 ) < 0;
    }
};

// map from strings to materials
typedef std::map< const char*, const Material*, StrCompare > MaterialMap;
// map from strings to meshes
typedef std::map< const char*, const Mesh*, StrCompare > MeshMap;
// map from strings to triangle vertices
typedef std::map< const char*, Triangle::Vertex, StrCompare > TriVertMap;
// map ints to bodies (physics)
typedef std::map< int, Body* > BodyMap;

static const char STR_FOV[] = "fov";
static const char STR_NEAR[] = "near_clip";
static const char STR_FAR[] = "far_clip";
static const char STR_POSITION[] = "position";
static const char STR_ORIENT[] = "orientation";
static const char STR_SCALE[] = "scale";
static const char STR_NORMAL[] = "normal";
static const char STR_TCOORD[] = "tex_coord";
static const char STR_COLOR[] = "color";
static const char STR_RADIUS[] = "radius";
static const char STR_VERTEX[] = "vertex";
static const char STR_ACON[] = "attenuation_constant";
static const char STR_ALIN[] = "attenuation_linear";
static const char STR_AQUAD[] = "attenuation_quadratic";
static const char STR_AMBIENT[] = "ambient";
static const char STR_DIFFUSE[] = "diffuse";
static const char STR_SPECULAR[] = "specular";
static const char STR_SHININESS[] = "shininess";
static const char STR_REFRACT[] = "refractive_index";
static const char STR_TEXTURE[] = "texture";
static const char STR_NAME[] = "name";
static const char STR_FILENAME[] = "filename";
static const char STR_BACKGROUND[] = "background_color";
static const char STR_AMLIGHT[] = "ambient_light";
static const char STR_CAMERA[] = "camera";
static const char STR_PLIGHT[] = "point_light";
static const char STR_MATERIAL[] = "material";
static const char STR_SPHERE[] = "sphere";
static const char STR_TRIANGLE[] = "triangle";
static const char STR_MODEL[] = "model";
static const char STR_MESH[] = "mesh";
static const char STR_BODY[] = "body";
static const char STR_ID[] = "id";
static const char STR_SPHEREBODY[] = "sphere_body";
static const char STR_TRIANGLEBODY[] = "triangle_body";
static const char STR_PLANEBODY[] = "plane_body";
static const char STR_POINTA[] = "point_a";
static const char STR_POINTB[] = "point_b";
static const char STR_POINTC[] = "point_c";
static const char STR_BODY1[] = "body1";
static const char STR_BODY1OFFSET[] = "body1_offset";
static const char STR_BODY2[] = "body2";
static const char STR_BODY2OFFSET[] = "body2_offset";
static const char STR_MASS[] = "mass";
static const char STR_VELOCITY[] = "velocity";
static const char STR_ANGULARVELOCITY[] = "angular_velocity";
static const char STR_GRAVITY[] = "gravity";
static const char STR_SPRING[] = "spring";
static const char STR_CONSTANT[] = "constant";
static const char STR_EQUILIBRIUM[] = "equilibrium";
static const char STR_OFFSET1[] = "offset1";
static const char STR_OFFSET2[] = "offset2";
static const char STR_COLLISIONDAMPING[] = "collision_damping";
static const char STR_DAMPING[] = "damping";

static void print_error_header( const TiXmlElement* base )
{
    std::cout << "ERROR, " << base->Row() << ":" << base->Column() << "; "
        << "in " << base->Value() << ", ";
}

static const TiXmlElement* get_unique_child( const TiXmlElement* parent, bool required, const char* name )
{
    const TiXmlElement* elem = parent->FirstChildElement( name );

    if ( !elem ) {
        if ( required ) {
            print_error_header( parent );
            std::cout << "no '" << name << "' defined.\n";
            throw std::exception();
        } else {
            return 0;
        }
    }

    if ( elem->NextSiblingElement( name ) ) {
        print_error_header( elem );
        std::cout << "'" << name << "' multiply defined.\n";
        throw std::exception();
    }

    return elem;
}

static void parse_attrib_int( const TiXmlElement* elem, bool required, const char* name, int* val )
{
    int rv = elem->QueryIntAttribute( name, val );
    if ( rv == TIXML_WRONG_TYPE ) {
        print_error_header( elem );
        std::cout << "error parsing '" << name << "'.\n";
        throw std::exception();
    } else if ( required && rv == TIXML_NO_ATTRIBUTE ) {
        print_error_header( elem );
        std::cout << "missing '" << name << "'.\n";
        throw std::exception();
    }
}

static void parse_attrib_double( const TiXmlElement* elem, bool required, const char* name, double* val )
{
    int rv = elem->QueryDoubleAttribute( name, val );
    if ( rv == TIXML_WRONG_TYPE ) {
        print_error_header( elem );
        std::cout << "error parsing '" << name << "'.\n";
        throw std::exception();
    } else if ( required && rv == TIXML_NO_ATTRIBUTE ) {
        print_error_header( elem );
        std::cout << "missing '" << name << "'.\n";
        throw std::exception();
    }
}

static void parse_attrib_string( const TiXmlElement* elem, bool required, const char* name, const char** val )
{
    const char* att = elem->Attribute( name );
    if ( !att && required ) {
        print_error_header( elem );
        std::cout << "missing '" << name << "'.\n";
        throw std::exception();
    } else if ( att ) {
        *val = att;
    }
}

static void parse_attrib_string( const TiXmlElement* elem, bool required, const char* name, std::string* val )
{
    const char* att = 0;
    parse_attrib_string( elem, required, name, &att );
    if ( att ) {
        *val = att;
    }
}

template< typename T >
static void parse_elem( const TiXmlElement* elem, T* val )
{
    throw std::exception();
}

template<> void parse_elem< int >( const TiXmlElement* elem, int* i )
{
    parse_attrib_int( elem, true, "i", i );
}

template<> void parse_elem< double >( const TiXmlElement* elem, double* d )
{
    parse_attrib_double( elem, true, "v", d );
}

template<> void parse_elem< Color3 >( const TiXmlElement* elem, Color3* color )
{
    parse_attrib_double( elem, true, "r", &color->r );
    parse_attrib_double( elem, true, "g", &color->g );
    parse_attrib_double( elem, true, "b", &color->b );
}

template<> void parse_elem< Vector2 >( const TiXmlElement* elem, Vector2* vector )
{
    // parse as if they were texture coordinates
    parse_attrib_double( elem, true, "u", &vector->x );
    parse_attrib_double( elem, true, "v", &vector->y );
}

template<> void parse_elem< Vector3 >( const TiXmlElement* elem, Vector3* vector )
{
    parse_attrib_double( elem, true, "x", &vector->x );
    parse_attrib_double( elem, true, "y", &vector->y );
    parse_attrib_double( elem, true, "z", &vector->z );
}

template<> void parse_elem< Quaternion >( const TiXmlElement* elem, Quaternion* quat )
{
    real_t x,y,z; // axis
    real_t a;     // angle
    parse_attrib_double( elem, true, "a", &a );
    parse_attrib_double( elem, true, "x", &x );
    parse_attrib_double( elem, true, "y", &y );
    parse_attrib_double( elem, true, "z", &z );
    *quat = Quaternion( Vector3( x, y, z ), a );
}

template< typename T >
static void parse_elem( const TiXmlElement* parent, bool required, const char* name, T* val )
{
    const TiXmlElement* child = get_unique_child( parent, required, name );
    if ( child )
        parse_elem< T >( child, val );
}

static void parse_camera( const TiXmlElement* elem, Camera* camera )
{
    Quaternion ori = camera->orientation;

    // note: we don't load aspect, since it's set by the application
    parse_elem( elem, true,  STR_FOV,       &camera->fov );
    parse_elem( elem, true,  STR_NEAR,      &camera->near_clip );
    parse_elem( elem, true,  STR_FAR,       &camera->far_clip );
    parse_elem( elem, true,  STR_POSITION,  &camera->position );
    parse_elem( elem, true,  STR_ORIENT,    &ori );
    // normalize orientation
    camera->orientation = normalize( ori );
}

static void parse_point_light( const TiXmlElement* elem, PointLight* light )
{
    parse_elem( elem, false, STR_ACON,      &light->attenuation.constant );
    parse_elem( elem, false, STR_ALIN,      &light->attenuation.linear );
    parse_elem( elem, false, STR_AQUAD,     &light->attenuation.quadratic );
    parse_elem( elem, true,  STR_POSITION,  &light->position );
    parse_elem( elem, true,  STR_COLOR,     &light->color );
}

template< typename T >
static void parse_lookup_data( const std::map< const char*, T, StrCompare > tmap, const TiXmlElement* elem, const char* name, T* val )
{
    typename std::map< const char*, T, StrCompare >::const_iterator iter;
    const char* att;

    parse_attrib_string( elem, true, name, &att );
    iter = tmap.find( att );
    if ( iter == tmap.end() ) {
        print_error_header( elem );
        std::cout << "No such " << name << " '" << att << "'.\n";
        throw std::exception();
    }
    *val = iter->second;
}

static const char* parse_material( const TiXmlElement* elem, Material* material )
{
    const char* name;

    parse_attrib_string( elem, false, STR_TEXTURE,  &material->texture_filename );
    parse_attrib_string( elem, true,  STR_NAME,     &name );

    parse_elem( elem, false, STR_REFRACT,   &material->refractive_index );
    parse_elem( elem, false, STR_AMBIENT,   &material->ambient );
    parse_elem( elem, false, STR_DIFFUSE,   &material->diffuse );
    parse_elem( elem, false, STR_SPECULAR,  &material->specular );
    parse_elem( elem, false, STR_SHININESS,  &material->shininess );

    return name;
}

static const char* parse_mesh( const TiXmlElement* elem, Mesh* mesh )
{
    const char* name;

    parse_attrib_string( elem, false, STR_FILENAME, &mesh->filename );
    parse_attrib_string( elem, true,  STR_NAME,     &name );

    return name;
}

static const char* parse_triangle_vertex( const MaterialMap& matmap, const TiXmlElement* elem, Triangle::Vertex* vertex )
{
    const char* name;
    Vector3 normal = vertex->normal;

    parse_elem( elem, true,  STR_POSITION,  &vertex->position );
    parse_elem( elem, true,  STR_NORMAL,    &normal );
    parse_elem( elem, true,  STR_TCOORD,    &vertex->tex_coord );
    parse_lookup_data( matmap, elem, STR_MATERIAL, &vertex->material );
    parse_attrib_string( elem, true,  STR_NAME,     &name );
    // normalize normal
    vertex->normal = normalize( normal );

    return name;
}

static void parse_geom_base( const MaterialMap& matmap, const TiXmlElement* elem, Geometry* geom )
{
    Quaternion ori = geom->orientation;

    parse_elem( elem, true,  STR_POSITION,  &geom->position );
    parse_elem( elem, false, STR_ORIENT,    &ori );
    parse_elem( elem, false, STR_SCALE,     &geom->scale );
    // normalize orientation
    geom->orientation = normalize( ori );
}

static void check_mem( void* ptr )
{
    if ( !ptr ) {
        std::cout << "Error: ran out of memory loading scene.\n";
        throw std::exception();
    }
}

static void parse_spherebody( const TiXmlElement* elem, SphereBody* body )
{
    parse_elem( elem, true, STR_ID, &body->id );
    parse_elem( elem, true, STR_MASS, &body->mass );
    parse_elem( elem, false, STR_POSITION, &body->position );
    parse_elem( elem, false, STR_RADIUS, &body->radius );
    parse_elem( elem, false, STR_VELOCITY, &body->velocity );
    parse_elem( elem, false, STR_ANGULARVELOCITY, &body->angular_velocity );
    parse_elem( elem, false, STR_ORIENT, &body->orientation );
}

static void parse_trianglebody( const TiXmlElement* elem, TriangleBody* body )
{
    parse_elem( elem, true, STR_ID, &body->id );
    parse_elem( elem, false, STR_POINTA, &body->vertices[0]);
    parse_elem( elem, false, STR_POINTB, &body->vertices[1]);
    parse_elem( elem, false, STR_POINTC, &body->vertices[2]);
    body->position = body->vertices[0];
}

static void parse_planebody( BodyMap& bodies, const TiXmlElement* elem, PlaneBody* body )
{
    parse_elem( elem, true, STR_ID, &body->id );
    parse_elem( elem, true, STR_POSITION, &body->position );
    parse_elem( elem, true, STR_NORMAL, &body->normal );
    bodies[body->id] = body;
}

static void parse_modelbody( const TiXmlElement* elem, ModelBody* body )
{
    parse_elem( elem, true, STR_ID, &body->id );
    parse_elem( elem, false, STR_POSITION, &body->position );
    parse_elem( elem, false, STR_ORIENT, &body->orientation );
}

static void parse_geom_sphere( const MaterialMap& matmap, BodyMap& bodies, Physics* phys, const TiXmlElement* elem, Sphere* geom )
{
    // parse base
    parse_geom_base( matmap, elem, geom );
    parse_elem( elem, true,  STR_RADIUS,  &geom->radius );
    parse_lookup_data( matmap, elem, STR_MATERIAL, &geom->material );
    const TiXmlElement* child = elem->FirstChildElement( STR_BODY );
    if ( child ) {
        SphereBody* body = new SphereBody( geom );
        check_mem( body );
        parse_spherebody( child, body );
        bodies[body->id] = body;
        phys->add_sphere(body);
    }
}

static void parse_geom_triangle( const MaterialMap& matmap, const TriVertMap& tvmap, BodyMap& bodies, Physics* phys, const TiXmlElement* elem, Triangle* geom )
{
    parse_geom_base( matmap, elem, geom );

    const TiXmlElement* child = elem->FirstChildElement( STR_VERTEX );
    size_t count = 0;
    while ( child ) {
        if ( count > 2 ) {
            print_error_header( child );
            std::cout << "Too many vertices for triangle.\n";
            throw std::exception();
        }

        parse_lookup_data( tvmap, child, STR_NAME, &geom->vertices[count] );

        child = child->NextSiblingElement( STR_VERTEX );
        count++;
    }

    if ( count < 2 ) {
        print_error_header( elem );
        std::cout << "Too few vertices for triangle.\n";
        throw std::exception();
    }
    child = elem->FirstChildElement( STR_BODY );
    if ( child ) {
        TriangleBody* body = new TriangleBody(geom);
        check_mem( body );
        parse_trianglebody( child, body );
        bodies[body->id] = body;
        phys->add_triangle(body);
    }
}

static void parse_geom_model( const MaterialMap& matmap, const MeshMap& meshmap, BodyMap& bodies, Physics* phys, const TiXmlElement* elem, Model* geom )
{
    parse_geom_base( matmap, elem, geom );
    parse_lookup_data( meshmap, elem, STR_MESH, &geom->mesh );
    parse_lookup_data( matmap, elem, STR_MATERIAL, &geom->material );
    const TiXmlElement* child = elem->FirstChildElement ( STR_BODY );
	if ( child ) {
        ModelBody* body = new ModelBody( geom );
        check_mem( body );
        parse_modelbody( child, body );
        bodies[body->id] = body;
        phys->add_model(body);
	}
}

static void parse_spring( BodyMap& bmap, const TiXmlElement* elem, Spring* spring )
{
    int id;
    parse_elem( elem, true, STR_CONSTANT, &spring->constant );
    parse_elem( elem, true, STR_EQUILIBRIUM, &spring->equilibrium );
    parse_elem( elem, true, STR_BODY1, &id );
    spring->body1 = bmap[id];
    parse_elem( elem, false, STR_OFFSET1, &spring->body1_offset );
    parse_elem( elem, true, STR_BODY2, &id );
    spring->body2 = bmap[id];
    parse_elem( elem, false, STR_OFFSET2, &spring->body2_offset );
    parse_elem( elem, false, STR_DAMPING, &spring->damping );
}

bool load_scene( Scene* scene, const char* filename )
{
    TiXmlDocument doc( filename );
    const TiXmlElement* root = 0;
    const TiXmlElement* elem = 0;
    MaterialMap materials;
    MeshMap meshes;
    TriVertMap triverts;
    BodyMap bodies;

    assert( scene );

    // load the document

    if ( !doc.LoadFile() ) {
        std::cout << "ERROR, " << doc.ErrorRow() << ":" << doc.ErrorCol() << "; "
            << "parse error: " << doc.ErrorDesc() << "\n";
        return false;
    }

    // check for root element

    root = doc.RootElement();
    if ( !root ) {
        std::cout << "No root element.\n";
        return false;
    }

    // reset the scene

    scene->reset();

    try {
        // parse the camera
        elem = get_unique_child( root, true, STR_CAMERA );
        parse_camera( elem, &scene->camera );
        // parse background color
        parse_elem( root, true,  STR_BACKGROUND, &scene->background_color );
        // parse refractive index
        parse_elem( root, true,  STR_REFRACT, &scene->refractive_index );
        // parse ambient light
        parse_elem( root, false, STR_AMLIGHT, &scene->ambient_light );
        // parse gravitational constant
        parse_elem( root, false, STR_GRAVITY, &scene->get_physics()->gravity );
		// parse damping constants
		parse_elem( root, false, STR_COLLISIONDAMPING, &scene->get_physics()->collision_damping );

        // parse the lights
        elem = root->FirstChildElement( STR_PLIGHT );
        while ( elem ) {
            PointLight pl;
            parse_point_light( elem, &pl );
            scene->add_light( pl );
            elem = elem->NextSiblingElement( STR_PLIGHT );
        }

        // parse the materials
        elem = root->FirstChildElement( STR_MATERIAL );
        while ( elem ) {
            Material* mat = new Material();
            check_mem( mat );
            scene->add_material( mat );
            const char* name = parse_material( elem, mat );
            assert( name );
            // place each material in map by it's name, so we can associate geometries
            // with them when loading geometries
            // check for repeat name
            if ( !materials.insert( std::make_pair( name, mat ) ).second ) {
                print_error_header( elem );
                std::cout << "Material '" << name << "' multiply defined.\n";
                throw std::exception();
            }
            elem = elem->NextSiblingElement( STR_MATERIAL );
        }

        // parse the meshes
        elem = root->FirstChildElement( STR_MESH );
        while ( elem ) {
            Mesh* mesh = new Mesh();
            check_mem( mesh );
            scene->add_mesh( mesh );
            const char* name = parse_mesh( elem, mesh );
            assert( name );
            // place each mesh in map by it's name, so we can associate geometries
            // with them when loading geometries
            if ( !meshes.insert( std::make_pair( name, mesh ) ).second ) {
                print_error_header( elem );
                std::cout << "Mesh '" << name << "' multiply defined.\n";
                throw std::exception();
            }
            elem = elem->NextSiblingElement( STR_MESH );
        }

        // parse vertices (used by triangles)
        elem = root->FirstChildElement( STR_VERTEX );
        while ( elem ) {
            Triangle::Vertex v;
            const char* name = parse_triangle_vertex( materials, elem, &v );
            assert( name );
            // place each vertex in map by it's name, so we can associate triangles
            // with them when loading geometries
            if ( !triverts.insert( std::make_pair( name, v ) ).second ) {
                print_error_header( elem );
                std::cout << "Triangle vertex '" << name << "' multiply defined.\n";
                throw std::exception();
            }
            elem = elem->NextSiblingElement( STR_VERTEX );
        }

        // parse the geometries

        // spheres
        elem = root->FirstChildElement( STR_SPHERE );
        while ( elem ) {
            Sphere* geom = new Sphere();
            check_mem( geom );
            scene->add_geometry( geom );
            parse_geom_sphere( materials, bodies, scene->get_physics(), elem, geom );
            elem = elem->NextSiblingElement( STR_SPHERE );
        }

        // triangles
        elem = root->FirstChildElement( STR_TRIANGLE );
        while ( elem ) {
            Triangle* geom = new Triangle();
            check_mem( geom );
            scene->add_geometry( geom );
            parse_geom_triangle( materials, triverts, bodies, scene->get_physics(), elem, geom );
            elem = elem->NextSiblingElement( STR_TRIANGLE );
        }

        // models
        elem = root->FirstChildElement( STR_MODEL );
        while ( elem ) {
            Model* geom = new Model();
            check_mem( geom );
			scene->add_geometry(geom);
			parse_geom_model(materials, meshes, bodies, scene->get_physics(), elem, geom);
			elem = elem->NextSiblingElement(STR_MODEL);
		}

        // TODO add you own geometries here

        // physical planes
        elem = root->FirstChildElement( STR_PLANEBODY );
        while ( elem ) {
            PlaneBody* body = new PlaneBody();
            check_mem( body );
            parse_planebody( bodies, elem, body );
            scene->get_physics()->add_plane( body );
            elem = elem->NextSiblingElement( STR_PLANEBODY );
        }

        // springs
        elem = root->FirstChildElement( STR_SPRING );
        while ( elem ) {
            Spring* spring = new Spring();
            check_mem( spring );
            parse_spring( bodies, elem, spring );
            scene->get_physics()->add_spring( spring );
            elem = elem->NextSiblingElement( STR_SPRING );
        }

    } catch ( std::bad_alloc const& ) {
        std::cout << "Out of memory error while loading scene\n.";
        scene->reset();
        return false;
    } catch ( ... ) {
        scene->reset();
        return false;
    }

    return true;

}

} /* _462 */

