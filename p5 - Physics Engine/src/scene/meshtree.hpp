//
//  MeshTree.h
//  p4
//
//  Created by Nathan Dobson on 11/11/14.
//  Copyright (c) 2014 Nathan Dobson. All rights reserved.
//

#ifndef __p4__MeshTree__
#define __p4__MeshTree__

#include <stdio.h>
#include <scene/scene.hpp>

#include "scene/mesh.hpp"
#include "scene/bound.hpp"

namespace _462 {

#define MAX_TREE_DEPTH 50

class MeshTreeNode{
public:
	MeshTreeNode();
	MeshTreeNode(const ModelBody *model_body, std::vector<MeshTriangle> triangles);
	~MeshTreeNode();

	Bound* bbox;
	MeshTreeNode* left;
	MeshTreeNode* right;
	
	std::vector<MeshTriangle> triangles;

	// get a bounding box surrounding all the triangles
	Bound* createBBox(const ModelBody *model_body);
	// get a bounding box from triangle
	Bound createTriangleBound(const ModelBody *model_body, MeshTriangle triangle);
};

class MeshTree{
public:
	MeshTreeNode* root;

	MeshTree();
	MeshTree(const ModelBody *model_body);
	~MeshTree();

	MeshTree *makeFlatTree(const ModelBody *model_body);
	void refine(const ModelBody *model_body, MeshTree *tree);
	MeshTree *makeRecTree(const ModelBody *model_body);
	void refineRec(const ModelBody *model_body, MeshTree *tree);
	Bound setBounds(const ModelBody *model_body, MeshTree *tree);

	void freeNode(MeshTreeNode* node);
	MeshTreeNode* build(const ModelBody *model_body, std::vector<MeshTriangle> triangles, int depth);
	Vector3 getTriangleMidPoint(const ModelBody *model_body, MeshTriangle triangle);
};
} //namespace _462
#endif /* defined(__p4__MeshTree__) */
