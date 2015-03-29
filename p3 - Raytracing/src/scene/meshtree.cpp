//
//  MeshTree.cpp
//  p4
//
//  Created by Nathan Dobson on 11/11/14.
//  Copyright (c) 2014 Nathan Dobson. All rights reserved.
//  A data structure for fast mesh/ray intersection test

#include "scene/meshtree.hpp"

#define MIN_REFINE_SIZE 2
#define MIN_REFINE_RATIO 2.0

namespace _462 {

MeshTreeNode::MeshTreeNode() {
	bbox = new Bound();
	left = NULL;
	right = NULL;
}

MeshTreeNode::MeshTreeNode(const Mesh* mesh, std::vector<MeshTriangle> triangles) {
	left = NULL;
	right = NULL;
	// copy triangles
	for (size_t i = 0; i < triangles.size(); ++i) {
		this->triangles.push_back(triangles[i]);
	}
	bbox = createBBox(mesh);
}

Bound* MeshTreeNode::createBBox(const Mesh* mesh) {
	Bound* bbox = new Bound();

	for (size_t i = 0; i < triangles.size(); ++i) {
		Bound bound = createTriangleBound(mesh, triangles[i]);
		bbox->expand(bound);
	}
	return bbox;
}

Bound MeshTreeNode::createTriangleBound(const Mesh* mesh, MeshTriangle triangle) {
	Vector3 min = mesh->vertices[triangle.vertices[0]].position;
	Vector3 max = mesh->vertices[triangle.vertices[0]].position;

	for (int i = 1; i < 3; ++i) {
		if (mesh->vertices[triangle.vertices[i]].position.x < min.x) min.x = mesh->vertices[triangle.vertices[i]].position.x;
		if (mesh->vertices[triangle.vertices[i]].position.y < min.y) min.y = mesh->vertices[triangle.vertices[i]].position.y;
		if (mesh->vertices[triangle.vertices[i]].position.z < min.z) min.z = mesh->vertices[triangle.vertices[i]].position.z;
		if (mesh->vertices[triangle.vertices[i]].position.x > max.x) max.x = mesh->vertices[triangle.vertices[i]].position.x;
		if (mesh->vertices[triangle.vertices[i]].position.y > max.y) max.y = mesh->vertices[triangle.vertices[i]].position.y;
		if (mesh->vertices[triangle.vertices[i]].position.z > max.z) max.z = mesh->vertices[triangle.vertices[i]].position.z;
	}

	return Bound(min, max);
}

MeshTreeNode::~MeshTreeNode() {
	delete bbox;
	delete left;
	delete right;
}

MeshTree::MeshTree() {
	root = NULL;
}

MeshTree::MeshTree(const Mesh* mesh) {
	std::vector<MeshTriangle> triangles;
	for (size_t i = 0; i < mesh->triangles.size(); ++i) {
		triangles.push_back(mesh->triangles[i]);
	}

	// create tree
	root = build(mesh, triangles, 1);
}

MeshTree::~MeshTree() {
	// delete all nodes in tree
	freeNode(root);
}

void MeshTree::freeNode(MeshTreeNode* node) {
	// Free the node postorder
	if (node != NULL)
	{
		freeNode(node->left);
		freeNode(node->right);
		delete node;
	}
}

Vector3 MeshTree::getTriangleMidPoint(const Mesh *mesh, MeshTriangle triangle) {
	Vector3 middlePoint = Vector3::Zero();

	for (int i = 0; i < 3; ++i) {
		middlePoint += mesh->vertices[triangle.vertices[i]].position * (real_t(1) / real_t(3));
	}

	return middlePoint;
}

MeshTreeNode* MeshTree::build(const Mesh *mesh, std::vector<MeshTriangle> triangles, int depth) {
	// end of recursion
	if (triangles.size() <= 0) {
		return NULL;
	}

	MeshTreeNode* node = new MeshTreeNode(mesh, triangles);

	// reach max depth or single triangle then create leaf
	if (triangles.size() == 1 || depth >= MAX_TREE_DEPTH) {
		return node;
	}

	// split triangles
	// find mid point of triangles
	Vector3 midPoint = Vector3::Zero();
	for (size_t i = 0; i < triangles.size(); ++i) {
		midPoint += getTriangleMidPoint(mesh, triangles[i]) * (real_t(1) / real_t(triangles.size()));
	}

	std::vector<MeshTriangle> leftTriangles;
	std::vector<MeshTriangle> rightTriangles;

	int longAxis = node->bbox->longestAxis();
	// split triangles based on their midpoints side of average in longest axist
	for (size_t i = 0; i < triangles.size(); ++i) {
		switch (longAxis) {
		case 0: // x-axis
			midPoint.x >= getTriangleMidPoint(mesh, triangles[i]).x ? rightTriangles.push_back(triangles[i]) : leftTriangles.push_back(triangles[i]);
			break;
		case 1: // y-axis
			midPoint.y >= getTriangleMidPoint(mesh, triangles[i]).y ? rightTriangles.push_back(triangles[i]) : leftTriangles.push_back(triangles[i]);
			break;
		case 2: // z-axis
			midPoint.z >= getTriangleMidPoint(mesh, triangles[i]).z ? rightTriangles.push_back(triangles[i]) : leftTriangles.push_back(triangles[i]);
			break;
		}
	}

	// cannot divide anymore
	if (leftTriangles.size() == 0 || rightTriangles.size() == 0) {
		return node;
	}

	node->left = build(mesh, leftTriangles, depth + 1);
	node->right = build(mesh, rightTriangles, depth + 1);
	
	return node;
}
} //namespace _462