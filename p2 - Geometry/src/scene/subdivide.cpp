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

	verticesSize = vertices.size();
	triangleSize = triangles.size();

	// INIT
	edgeMap.clear();
	neigMap.clear();

	// CREATE EDGE
	// iterate triangle to create edge
	for (unsigned int i = 0; i < triangleSize; ++i) {
		// create edges
		createEdge(triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]);
		createEdge(triangles[i].vertices[0], triangles[i].vertices[2], triangles[i].vertices[1]);
		createEdge(triangles[i].vertices[1], triangles[i].vertices[2], triangles[i].vertices[0]);
	}
	
	// CREATE ODD VERTICES
	//
	//          P1
	//		   / \
	//        /   \
	//		 n3----n1
	//	    / \   / \
	//     /   \ /   \
	//   P3-----n2---P2
	//
	for (unsigned int i = 0; i < triangleSize; ++i) {
		// add new vertices
		unsigned int n1 = createOddVertex(createKey(triangles[i].vertices[0], triangles[i].vertices[1]));
		unsigned int n2 = createOddVertex(createKey(triangles[i].vertices[1], triangles[i].vertices[2]));
		unsigned int n3 = createOddVertex(createKey(triangles[i].vertices[2], triangles[i].vertices[0]));

		unsigned int P1 = triangles[i].vertices[0];
		unsigned int P2 = triangles[i].vertices[1];
		unsigned int P3 = triangles[i].vertices[2];

		// modify old triangle to be center triangle
		triangles[i].vertices[0] = n1;
		triangles[i].vertices[1] = n3;
		triangles[i].vertices[2] = n2;
		// add new triangles
		triangles.push_back(createTriangle(P1, n3, n1));
		triangles.push_back(createTriangle(P2, n1, n2));
		triangles.push_back(createTriangle(P3, n2, n3));
	}

	// find neighbor for every vertices
	createNeigMap();

	// MODIFY EVEN VERTICES
	modifyEvenVertices();

	// RECALCULATE NORMAL
	computeNormal();

	create_gl_data();

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

void Mesh::createNeigMap() {
	// iterate edge
	for (auto i = edgeMap.begin(); i != edgeMap.end(); ++i) {
		Edge edge = i->second;
		// boundary edge
		if (edge.isBoundary) {
			neigMap[edge.mainVert[0]].neigBoundary.push_back(edge.mainVert[1]);
			neigMap[edge.mainVert[1]].neigBoundary.push_back(edge.mainVert[0]);
		}
		else { // interior edge
			neigMap[edge.mainVert[0]].neigInterior.push_back(edge.mainVert[1]);
			neigMap[edge.mainVert[1]].neigInterior.push_back(edge.mainVert[0]);
		}
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

void Mesh::modifyEvenVertices() {
	// allocate memory
	Vector3 *meshVertex = new Vector3[verticesSize];

	for (unsigned int i = 0; i < verticesSize; ++i) {
		NeigVertices neigVertices = neigMap[i];

		// boundary
		if (neigVertices.neigBoundary.size() > 0) {
			meshVertex[i] = (0.75 * vertices[i].position) + (0.125 * (vertices[neigVertices.neigBoundary[0]].position + vertices[neigVertices.neigBoundary[1]].position));
		}
		else { // interior
			double nSize = (double)neigVertices.neigInterior.size();
			double B = (1.0 / nSize) * (0.625 - pow(0.375 + (0.25 * cos(2.0*PI / nSize)), 2));

			Vector3 sumNeig = {0, 0, 0};
			for (int j = 0; j < (int)nSize; ++j)
				sumNeig += vertices[neigVertices.neigInterior[j]].position;

			meshVertex[i] = ((1.0 - (B * nSize)) * vertices[i].position) + (B * sumNeig);
		}
	}

	for (unsigned int i = 0; i < verticesSize; ++i) {
		vertices[i].position = meshVertex[i];
	}

	// freeing memory
	delete[] meshVertex;
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
