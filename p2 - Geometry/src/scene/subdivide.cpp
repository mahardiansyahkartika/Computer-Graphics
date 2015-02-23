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

	// generate odd vertices - first pass
	createOddVertices();
	// divide triangle into 4 new triangles
	divideTriangles();
	// modify even vertices - second pass
	modifyEvenVertices();

	// modify edges
	std::copy(newEdges, newEdges + newEdgesSize, edges);
	edgesSize = newEdgesSize;
	// clear newEdge list
	newEdgesSize = 0;

	// RECALCULATE NORMAL
	computeNormal();

	create_gl_data();

	return true;
}

void Mesh::addNeigMap(MeshVertex& meshVertex, bool isBoundary, unsigned int val) {
	if (meshVertex.totalNeigVert == 0) { // nothing inside
		meshVertex.isBoundary = isBoundary;
		meshVertex.neigMap[0] = val;
		meshVertex.totalNeigVert = 1;
	}
	else if (meshVertex.isBoundary == isBoundary) { // push back
		meshVertex.neigMap[meshVertex.totalNeigVert] = val;
		meshVertex.totalNeigVert++;
	}
	else if (isBoundary) { // if isBoundary then just add neigVert with boundary neighbour
		meshVertex.isBoundary = isBoundary;
		meshVertex.neigMap[0] = val;
		meshVertex.totalNeigVert = 1;
	}
	// if meshVertex.isBoundary and isBoundary == false then do nothing
}

void Mesh::createOddVertices() {
	for (unsigned int i = 0; i < edgesSize; ++i) {
		MeshVertex vertex;

		// boundary
		if (edges[i].totalTriangles == 1) {
			vertex.position = 0.5 * (vertices[edges[i].mainVert[0]].position + vertices[edges[i].mainVert[1]].position);
			// add neigMap
			addNeigMap(vertices[edges[i].mainVert[0]], true, edges[i].mainVert[1]);
			addNeigMap(vertices[edges[i].mainVert[1]], true, edges[i].mainVert[0]);
		}
		// interior
		else {
			vertex.position =
				(0.375 * (vertices[edges[i].mainVert[0]].position + vertices[edges[i].mainVert[1]].position)) +
				(0.125 * (vertices[edges[i].neigVert[0]].position + vertices[edges[i].neigVert[1]].position));
			// add neigMap
			addNeigMap(vertices[edges[i].mainVert[0]], false, edges[i].mainVert[1]);
			addNeigMap(vertices[edges[i].mainVert[1]], false, edges[i].mainVert[0]);
		}

		// save vertices index
		edges[i].oddVertexId = vertices.size();
		// push to vertices
		vertices.push_back(vertex);
		
		// add children edges. 1 edge will be 2 edges
		// 1st child
		Edge firstEdge = createEdge(edges[i].mainVert[0], edges[i].oddVertexId, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, 0, EMPTY);
		edges[i].children[0] = newEdgesSize; // set child index
		newEdges[newEdgesSize] = firstEdge; // push as new edge
		newEdgesSize++;
		// 2nd child
		Edge secondEdge = createEdge(edges[i].oddVertexId, edges[i].mainVert[1], EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, 0, EMPTY);
		edges[i].children[1] = newEdgesSize; // set child index
		newEdges[newEdgesSize] = secondEdge; // push as new edge
		newEdgesSize++;
	}
}

void Mesh::divideTriangles() {
	// New Triangles
	// from 1 triangle will create new 3 triangle (1,2,3)
	// old triangle will be triangle 4
	//
	//          P[0]
	//    e[0]  / \  e[5]
	//         / 1 \
	// 	    n[0]----n[2]
	//  e[1] / \ 4 / \  e[4]
	//      / 2 \ / 3 \
	//   P[1]---n[1]---P[2]
	//       e[2]   e[3]

	for (unsigned int i = 0; i < triangleSize; ++i) {
		// odd vertices
		unsigned int n[] = { edges[triangles[i].edges[0]].oddVertexId, edges[triangles[i].edges[1]].oddVertexId, edges[triangles[i].edges[2]].oddVertexId };
		// old vertices
		unsigned int P[] = { triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2] };
		// outer edges
		unsigned int e[6];
		for (unsigned int j = 0; j < 3; ++j) {
			int id1 = 0, id2 = 1;
			if (P[j] != edges[triangles[i].edges[j]].mainVert[0]) {
				id1 = 1; id2 = 0;
			}
			e[j * 2] = edges[triangles[i].edges[j]].children[id1];
			e[(j * 2) + 1] = edges[triangles[i].edges[j]].children[id2];
		}

		// TRIANGLE 1
		int edgeInside1Idx = newEdgesSize;
		Edge edgeInside1 = createEdge(n[2], n[0], P[0], n[1], triangles.size(), i, EMPTY, EMPTY, 2, EMPTY);
		MeshTriangle triangle1 = createTriangle(P[0], n[0], n[2], e[0], edgeInside1Idx, e[5]);
		// modify outer edges (e[0] & e[5])
		newEdges[e[0]].neigVert[newEdges[e[0]].totalTriangles] = n[2];
		newEdges[e[0]].triangles[newEdges[e[0]].totalTriangles] = triangles.size();
		newEdges[e[0]].totalTriangles++;
		newEdges[e[5]].neigVert[newEdges[e[5]].totalTriangles] = n[0];
		newEdges[e[5]].triangles[newEdges[e[5]].totalTriangles] = triangles.size();
		newEdges[e[5]].totalTriangles++;
		// push to vector
		triangles.push_back(triangle1);
		newEdges[newEdgesSize] = edgeInside1;
		newEdgesSize++;

		// TRIANGLE 2
		int edgeInside2Idx = newEdgesSize;
		Edge edgeInside2 = createEdge(n[0], n[1], P[1], n[2], triangles.size(), i, EMPTY, EMPTY, 2, EMPTY);
		MeshTriangle triangle2 = createTriangle(P[1], n[1], n[0], e[2], edgeInside2Idx, e[1]);
		// modify outer edges (e[1] & e[2])
		newEdges[e[1]].neigVert[newEdges[e[1]].totalTriangles] = n[1];
		newEdges[e[1]].triangles[newEdges[e[1]].totalTriangles] = triangles.size();
		newEdges[e[1]].totalTriangles++;
		newEdges[e[2]].neigVert[newEdges[e[2]].totalTriangles] = n[0];
		newEdges[e[2]].triangles[newEdges[e[2]].totalTriangles] = triangles.size();
		newEdges[e[2]].totalTriangles++;
		// push to vector
		triangles.push_back(triangle2);
		newEdges[newEdgesSize] = edgeInside2;
		newEdgesSize++;

		// TRIANGLE 3
		int edgeInside3Idx = newEdgesSize;
		Edge edgeInside3 = createEdge(n[1], n[2], P[2], n[0], triangles.size(), i, EMPTY, EMPTY, 2, EMPTY);
		MeshTriangle triangle3 = createTriangle(P[2], n[2], n[1], e[4], edgeInside3Idx, e[3]);
		// modify outer edges (e[3] & e[4])
		newEdges[e[3]].neigVert[newEdges[e[3]].totalTriangles] = n[2];
		newEdges[e[3]].triangles[newEdges[e[3]].totalTriangles] = triangles.size();
		newEdges[e[3]].totalTriangles++;
		newEdges[e[4]].neigVert[newEdges[e[4]].totalTriangles] = n[1];
		newEdges[e[4]].triangles[newEdges[e[4]].totalTriangles] = triangles.size();
		newEdges[e[4]].totalTriangles++;
		// push to vector
		triangles.push_back(triangle3);
		newEdges[newEdgesSize] = edgeInside3;
		newEdgesSize++;

		// TRIANGLE 4
		triangles[i].vertices[0] = n[2];
		triangles[i].vertices[1] = n[0];
		triangles[i].vertices[2] = n[1];
		triangles[i].edges[0] = edgeInside1Idx;
		triangles[i].edges[1] = edgeInside2Idx;
		triangles[i].edges[2] = edgeInside3Idx;
	}
}

void Mesh::modifyEvenVertices() {
	// allocate memory
	Vector3 *meshVertex = new Vector3[verticesSize];

	for (unsigned int i = 0; i < verticesSize; ++i) {
		// boundary
		if (vertices[i].isBoundary) {
			meshVertex[i] = (0.75 * vertices[i].position) + (0.125 * (vertices[vertices[i].neigMap[0]].position + vertices[vertices[i].neigMap[1]].position));
		}
		else { // interior
			double nSize = (double)vertices[i].totalNeigVert;
			double B = bValues[vertices[i].totalNeigVert - 1];

			Vector3 sumNeig = { 0, 0, 0 };
			for (unsigned int j = 0; j < vertices[i].totalNeigVert; ++j)
				sumNeig += vertices[vertices[i].neigMap[j]].position;

			meshVertex[i] = ((1.0 - (B * nSize)) * vertices[i].position) + (B * sumNeig);
		}
		// init triangle size
		vertices[i].totalNeigVert = 0;
	}

	for (unsigned int i = 0; i < verticesSize; ++i) {
		vertices[i].position = meshVertex[i];
	}

	// freeing memory
	delete[] meshVertex;
}

double Mesh::getB(unsigned int nSize) {
	return (1.0 / nSize) * (0.625 - pow(0.375 + (0.25 * cos(2.0*PI / nSize)), 2));
}

void Mesh::generateFirstEdgeList() {
	std::unordered_map<std::string, Edge> edgeMap;

	// store all B values
	for (unsigned int i = 1; i <= sizeof(bValues) / sizeof(bValues[0]); ++i) {
		bValues[i - 1] = getB(i);
	}

	for (unsigned int i = 0; i < triangles.size(); ++i) {
		// create edges
		generateEdge(i, triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2], edgeMap);
		generateEdge(i, triangles[i].vertices[0], triangles[i].vertices[2], triangles[i].vertices[1], edgeMap);
		generateEdge(i, triangles[i].vertices[1], triangles[i].vertices[2], triangles[i].vertices[0], edgeMap);
	}

	// allocate memory
	edges = new Edge[edgesCapacity];
	newEdges = new Edge[edgesCapacity];

	// reserve memory for vertices avoiding bad allocation, for 6th iteration stegosaurus vertices.size() is around ~2million. I reserve 3million instead
	// if you iterate more than 6 times in stegosaurus/teapot2, it will error cos I need to reserve more than 3million
	vertices.reserve(3000000);

	// convert from hashmap to array
	for (auto i = edgeMap.begin(); i != edgeMap.end(); ++i) {
		std::string key = i->first;
		Edge edge = i->second;

		// set edge index to triangles
		for (unsigned int j = 0; j < edge.totalTriangles; j++) {
			MeshTriangle triangle = triangles[edge.triangles[j]];
			int index;
			if (createKey(triangle.vertices[0], triangle.vertices[1]) == key) index = 0;
			else if (createKey(triangle.vertices[1], triangle.vertices[2]) == key) index = 1;
			else index = 2;

			triangles[edge.triangles[j]].edges[index] = edgesSize;
		}

		// push to vector
		edges[edgesSize] = edge;
		edgesSize++;
	}
}

void Mesh::generateEdge(int triangleIndex, int id1, int id2, int idNeighbor, std::unordered_map<std::string, Edge> &edgeMap) {
	// create key
	std::string key = createKey(id1, id2);

	// create new
	if (edgeMap.find(key) == edgeMap.end()) {
		Edge edge = createEdge(id1, id2, idNeighbor, EMPTY, triangleIndex, EMPTY, EMPTY, EMPTY, 1, EMPTY);
		edgeMap[key] = edge;
	}
	else { // modify existing edge
		Edge &edge = edgeMap[key];
		edge.neigVert[1] = idNeighbor;
		edge.triangles[1] = triangleIndex;
		edge.totalTriangles++;
	}
}

Edge Mesh::createEdge(int mainVert1, int mainVert2, int neigVert1, int neigVert2, int triangle1, int triangle2, int children1, int children2, unsigned int totalTriangles, int oddVertexId) {
	Edge edge;
	edge.mainVert[0] = mainVert1; edge.mainVert[1] = mainVert2;
	edge.neigVert[0] = neigVert1; edge.neigVert[1] = neigVert2;
	edge.triangles[0] = triangle1; edge.triangles[1] = triangle2;
	edge.children[0] = children1; edge.children[1] = children2;
	edge.totalTriangles = totalTriangles;
	edge.oddVertexId = oddVertexId;
	return edge;
}

MeshTriangle Mesh::createTriangle(int vertex1, int vertex2, int vertex3, int edge1, int edge2, int edge3) {
	MeshTriangle triangle;

	triangle.vertices[0] = vertex1; triangle.vertices[1] = vertex2; triangle.vertices[2] = vertex3;
	triangle.edges[0] = edge1; triangle.edges[1] = edge2; triangle.edges[2] = edge3;

	return triangle;
}

std::string Mesh::createKey(int id1, int id2) {
	std::stringstream ss;
	id1 < id2 ? ss << id1 << '.' << id2 : ss << id2 << '.' << id1;
	return ss.str();
}

void Mesh::computeNormal() {
	// initialize previous normal
	for (unsigned int i = 0; i < vertices.size(); ++i)
		vertices[i].normal = { 0, 0, 0 };

	// calculate normal based on triangle
	for (unsigned int i = 0; i < triangles.size(); ++i) {
		// normal vector = (b-a) x (c-a)
		Vector3 a = vertices[triangles[i].vertices[0]].position;
		Vector3 b = vertices[triangles[i].vertices[1]].position;
		Vector3 c = vertices[triangles[i].vertices[2]].position;

		Vector3 normal = normalize(cross((b - a), (c - a)));

		// assign the normal to the vertices
		for (unsigned int j = 0; j < 3; j++)
			vertices[triangles[i].vertices[j]].normal += normal;
	}

	// normalize the normal
	for (unsigned int i = 0; i < vertices.size(); ++i) {
		vertices[i].normal = normalize(vertices[i].normal);
	}
}

} /* _462 */
