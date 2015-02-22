/**
 * @file mesh.hpp
 * @brief Mesh class and OBJ loader.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#ifndef _462_SCENE_MESH_HPP_
#define _462_SCENE_MESH_HPP_
#define EMPTY -1

#include "math/vector.hpp"

#include <vector>
#include <cassert>
#include <unordered_map>
#include <sstream>

namespace _462 {

struct MeshVertex
{
    //Vector4 color;
    Vector3 position;
    Vector3 normal;
    Vector2 tex_coord;
};

struct MeshTriangle
{
    // index into the vertex list of the 3 vertices
    unsigned int vertices[3];
	// index into the edge list
	unsigned int edges[3];
};

struct Edge
{	
	// main vertices
	int mainVert[2];
	// neighbour vertices
	int neigVert[2];
	// triangles
	int triangles[2];
	// children edge index
	int children[2];
	// total triangles
	unsigned int totalTriangles = 0;
	// odd vertex index
	int oddVertexId = EMPTY;
};

struct NeigVertices
{
	std::vector<int> neigBoundary;
	std::vector<int> neigInterior;
};

/**
 * A mesh of triangles.
 */
class Mesh
{
public:

    Mesh();
    virtual ~Mesh();

    typedef std::vector< MeshTriangle > MeshTriangleList;
    typedef std::vector< MeshVertex > MeshVertexList;

    // The list of all triangles in this model.
    MeshTriangleList triangles;

    // The list of all vertices in this model.
    MeshVertexList vertices;

    // scene loader stores the filename of the mesh here
    std::string filename;

    bool has_tcoords;
    bool has_normals;
    int has_colors;

	// additional attributes
	unsigned int verticesSize, triangleSize, edgesSize;
	std::unordered_map<int, NeigVertices> neigMap;
	std::vector<Edge> edges;

    // Loads the model into a list of triangles and vertices.
    bool load();

    // Creates opengl data for rendering and computes normals if needed
    bool create_gl_data();

    bool subdivide();

    // Renders the mesh using opengl.
    void render() const;

	// additional functions / procedures
	void generateFirstEdgeList(); // called once
	void generateEdge(int triangleIndex, int id1, int id2, int idNeighbor, std::unordered_map<std::string, Edge> &edgeMap);
	std::string createKey(int id1, int id2);

	void createOddVertices();
	void divideTriangles();
	void modifyEvenVertices();
	Edge createEdge(int mainVert1, int mainVert2, int neigVert1, int neigVert2, int triangle1, int triangle2, int children1, int children2, unsigned int totalTriangles, int oddVertexId);
	MeshTriangle createTriangle(int vertex1, int vertex2, int vertex3, int edge1, int edge2, int edge3);
	void computeNormal();

private:
    typedef std::vector< float > FloatList;
    typedef std::vector< unsigned int > IndexList;

    // the vertex data used for GL rendering
    FloatList vertex_data;
    // the index data used for GL rendering
    IndexList index_data;

    // prevent copy/assignment
    Mesh( const Mesh& );
    Mesh& operator=( const Mesh& );

};


} /* _462 */

#endif /* _462_SCENE_MESH_HPP_ */
