/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "application/opengl.hpp"
#include "scene/model.hpp"
#include "scene/material.hpp"
#include "scene/meshtree.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ) { }
Model::~Model() { 
	delete tree;
}

bool Model::initialize(){
	Geometry::initialize();

	// create tree
	std::cout << "Start creating Tree" << std::endl;
	tree = new MeshTree(mesh, mat);
	std::cout << "Done creating Tree" << std::endl;

	return true;
}

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}

} /* _462 */

