#include "scene/mesh.hpp"

namespace _462 {

bool Mesh::subdivide()
{
    /*
      You should implement loop subdivision here.

      Triangles are stored in an std::vector<MeshTriangle> in 'triangles'.
      Vertices are stored in an std::vector<MeshVertex> in 'vertices'.

      Check mesh.hpp for the Mesh class definition.
	*/

	int verticesSize = vertices.size();
	int triangleSize = triangles.size();

	// CREATE EDGE
	edgeMap.clear();
	// iterate triangle to create edge
	for (unsigned int i = 0; i < triangleSize; ++i) {
		// create edges
		createEdge(triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]);
		createEdge(triangles[i].vertices[0], triangles[i].vertices[2], triangles[i].vertices[1]);
		createEdge(triangles[i].vertices[1], triangles[i].vertices[2], triangles[i].vertices[0]);
	}

	// CREATE ODD VERTICES
	/*
	           P1
			  / \
	         /   \
			n3----n1
		   / \   / \
		  /   \ /   \
	    P3-----n2---P2
	*/
	for (unsigned int i = 0; i < triangleSize; ++i) {
		// add new vertices
		int n1 = createOddVertex(createKey(triangles[i].vertices[0], triangles[i].vertices[1]));
		int n2 = createOddVertex(createKey(triangles[i].vertices[1], triangles[i].vertices[2]));
		int n3 = createOddVertex(createKey(triangles[i].vertices[2], triangles[i].vertices[0]));

		int P1 = triangles[i].vertices[0];
		int P2 = triangles[i].vertices[1];
		int P3 = triangles[i].vertices[2];

		// modify ol triangle to be center triangle
		triangles[i].vertices[0] = n1;
		triangles[i].vertices[1] = n2;
		triangles[i].vertices[2] = n3;
		// add new triangles
		triangles.push_back(createTriangle(P1, n1, n3));
		triangles.push_back(createTriangle(P2, n2, n1));
		triangles.push_back(createTriangle(P3, n3, n2));
	}

	// MODIFY EVEN VERTICES
	for (unsigned int i = 0; i < verticesSize; ++i) {
		
	}

	// RECALCULATE NORMAL
	computeNormal();

	return true;
}

void Mesh::createEdge(int id1, int id2, int idNeighbor) {
	// create key
	std::string key = createKey(id1, id2);

	// create new
	if (edgeMap.find(key) == edgeMap.end()) {
		Edge edge;
		edge.isBoundary = true;
		edge.mainVert[0] = id1;
		edge.mainVert[1] = id2;
		edge.neigVert[0] = idNeighbor;

		edgeMap[key] = edge;
	}
	else { // modify existing edge
		Edge &edge = edgeMap[key];
		edge.isBoundary = false;
		edge.neigVert[1] = idNeighbor;
	}
}

int Mesh::createOddVertex(std::string key) {
	if (edgeMap[key].oddVertexId == -1) {
		MeshVertex vertex;
		// boundary
		if (edgeMap[key].isBoundary) {
			vertex.position = 0.5 * (vertices[edgeMap[key].mainVert[0]].position + vertices[edgeMap[key].mainVert[1]].position);
		}
		else { // interior
			vertex.position =
				(0.375 * (vertices[edgeMap[key].mainVert[0]].position + vertices[edgeMap[key].mainVert[1]].position)) +
				(0.125 * (vertices[edgeMap[key].neigVert[0]].position + vertices[edgeMap[key].neigVert[1]].position));
		}
		// push to vertices
		vertices.push_back(vertex);
		// save vertices index
		int verticesIndex = vertices.size() - 1;
		edgeMap[key].oddVertexId = verticesIndex;
	}
	
	return edgeMap[key].oddVertexId;
}

MeshTriangle Mesh::createTriangle(int id1, int id2, int id3) {
	MeshTriangle triangle;
	triangle.vertices[0] = id1;
	triangle.vertices[1] = id2;
	triangle.vertices[2] = id3;
	
	return triangle;
}

std::string Mesh::createKey(int id1, int id2) {
	std::stringstream ss;
	if (id1 < id2) ss << id1 << '.' << id2;
	else ss << id2 << '.' << id1;
	return ss.str();
}

void Mesh::computeNormal() {
	// initialize previous normal
	for (unsigned int i = 0; i < vertices.size(); i++)
		vertices[i].normal = {0, 0, 0};

	// calculate normal based on triangle
	for (unsigned int i = 0; i < triangles.size(); ++i) {
		// normal vector = (b-a) x (c-a)
		Vector3 a = vertices[triangles[i].vertices[0]].position;
		Vector3 b = vertices[triangles[i].vertices[1]].position;
		Vector3 c = vertices[triangles[i].vertices[2]].position;

		Vector3 normal = normalize(cross((b - a), (c - a)));

		// assign the normal to the vertices
		for (unsigned int j = 0; j < 3; j++) {
			vertices[triangles[i].vertices[j]].normal += normal;
		}
	}

	// normalize the normal
	for (unsigned int i = 0; i < vertices.size(); i++) {
		vertices[i].normal = normalize(vertices[i].normal);
	}
}

} /* _462 */
