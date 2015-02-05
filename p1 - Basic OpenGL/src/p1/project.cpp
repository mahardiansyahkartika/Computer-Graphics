/**
 * @file project.cpp
 * @brief OpenGL project
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "p1/project.hpp"

// use this header to include the OpenGL headers
// DO NOT include gl.h or glu.h directly; it will not compile correctly.
#include "application/opengl.hpp"

// A namespace declaration. All proejct files use this namespace.
// Add this declration (and its closing) to all source/headers you create.
// Note that all #includes should be BEFORE the namespace declaration.
namespace _462 {

// definitions of functions for the OpenglProject class

// constructor, invoked when object is created
OpenglProject::OpenglProject()
{
    // TODO any basic construction or initialization of members
    // Warning: Although members' constructors are automatically called,
    // ints, floats, pointers, and classes with empty contructors all
    // will have uninitialized data!
}

// destructor, invoked when object is destroyed
OpenglProject::~OpenglProject()
{
    // TODO any final cleanup of members
    // Warning: Do not throw exceptions or call virtual functions from deconstructors!
    // They will cause undefined behavior (probably a crash, but perhaps worse).
}

/**
 * Initialize the project, doing any necessary opengl initialization.
 * @param camera An already-initialized camera.
 * @param scene The scene to render.
 * @return true on success, false on error.
 */
bool OpenglProject::initialize( Camera* camera, Scene* scene )
{
	// TODO opengl initialization code and precomputation of mesh/heightmap
	// init glew
	glewInit();

	mat = new Matrix4;

	// avoid mess up normals from Transformation
	glEnable(GL_NORMALIZE);

    // copy scene
    this->scene = *scene;

	// set light
	initLight();

	// set the camera
	setCamera(camera);

	// initialize for pool
	// allocate memory for normal pool mesh
	this->scene.mesh.normals = new Vector3[this->scene.mesh.num_vertices];
	// compute mesh normal
	computeMeshNormals(this->scene.mesh);

	// initialize for water
	generateWaterMesh(128);

	return true;
}

/**
 * Clean up the project. Free any memory, etc.
 */
void OpenglProject::destroy()
{
    // TODO any cleanup code, e.g., freeing memory
	delete mat;
	// freeing memory from mesh normals
	delete[] scene.mesh.normals;
	// freeing memory from heightmap
	delete[] scene.heightmap->mesh.vertices;
	delete[] scene.heightmap->mesh.triangles;
	delete[] scene.heightmap->mesh.normals;
}

/**
 * Perform an update step. This happens on a regular interval.
 * @param dt The time difference from the previous frame to the current.
 */
void OpenglProject::update( real_t dt )
{
    // update our heightmap
    scene.heightmap->update( dt );

    // TODO any update code, e.g. commputing heightmap mesh positions and normals
	// update water vertices
	for (unsigned int i = 0; i < scene.heightmap->mesh.num_vertices; i++) {
		scene.heightmap->mesh.vertices[i].y = scene.heightmap->compute_height(Vector2(scene.heightmap->mesh.vertices[i].x, scene.heightmap->mesh.vertices[i].z));
	}
	// update normal
	computeMeshNormals(scene.heightmap->mesh);
}

/**
 * Clear the screen, then render the mesh using the given camera.
 * @param camera The logical camera to use.
 * @see math/camera.hpp
 */
void OpenglProject::render( const Camera* camera )
{
    // TODO render code
	// clear the buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	setCamera(camera);

	glMatrixMode(GL_MODELVIEW); // set current matrix
	glLoadIdentity();  // Clears the matrix

	gluLookAt(
		camera->get_position().x, camera->get_position().y, camera->get_position().z,
		camera->get_direction().x, camera->get_direction().y, camera->get_direction().z,
		camera->get_up().x, camera->get_up().y, camera->get_up().z
		);

	// render pool & water
	Vector4 poolColor = Vector4(255.0 / 255.0, 20.0 / 255.0, 0.0 / 255.0, 1.0);
	Vector4 waterColor = Vector4(0.0 / 255.0, 100.0 / 255.0, 200.0 / 255.0, 1.0);
	float mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float shininess = 10;

	renderMeshWithMaterial(scene.mesh, scene.mesh_position, poolColor, mat_specular, shininess); // pool
	renderMeshWithMaterial(scene.heightmap->mesh, scene.heightmap_position, waterColor, mat_specular, shininess); // water
}

void OpenglProject::renderMeshWithMaterial(MeshData& mesh, PositionData& mesh_position, Vector4 color, float matSpecular[], float shininess) {
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glColor4f(color.x, color.y, color.z, color.w);

	glMaterialfv(GL_FRONT, GL_SPECULAR, matSpecular);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	renderMesh(mesh, mesh_position);
	glDisable(GL_COLOR_MATERIAL);
}

void OpenglProject::renderMesh(MeshData& mesh, PositionData& mesh_position) {
	// apply transforms   
	glPushMatrix();
	make_transformation_matrix(mat, mesh_position.position, mesh_position.orientation, mesh_position.scale);
	glMultMatrixd(mat->m);

	/* DRAW OBJECT (VBO Method) */
	// convert vertices and triangle data
	GLuint vertId, idxId;
	GLfloat* vertices;
	GLuint* indices;

	std::vector<GLfloat> vectorVertices;
	for (unsigned i = 0; i < mesh.num_vertices; i++) {
		// index 0-2 = vertex
		vectorVertices.push_back(mesh.vertices[i].x);
		vectorVertices.push_back(mesh.vertices[i].y);
		vectorVertices.push_back(mesh.vertices[i].z);
		// index 3-5 = normal
		vectorVertices.push_back(mesh.normals[i].x);
		vectorVertices.push_back(mesh.normals[i].y);
		vectorVertices.push_back(mesh.normals[i].z);
	}
	// total data contained in one vertices
	unsigned int totalDataInSingleVertex = 6;
	
	// convert vector to array
	vertices = &vectorVertices[0];

	std::vector<GLuint> vectorIndices;
	for (unsigned int i = 0; i < mesh.num_triangles; i++)
		for (unsigned int j = 0; j < 3; j++)
			vectorIndices.push_back(mesh.triangles[i].vertices[j]);
	// convert vector to array
	indices = &vectorIndices[0];

	// Initialize
	glGenBuffers(1, &vertId);
	glBindBuffer(GL_ARRAY_BUFFER, vertId);
	glBufferData(GL_ARRAY_BUFFER, mesh.num_vertices * totalDataInSingleVertex * sizeof(GLfloat), vertices, GL_STREAM_DRAW);

	glGenBuffers(1, &idxId);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idxId);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.num_triangles * 3 * sizeof(GLuint), indices, GL_STATIC_DRAW);

	// drawing function
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, totalDataInSingleVertex * sizeof(GLfloat), BUFFER_OFFSET(0)); //The starting point of the VBO, for the vertices
	glNormalPointer(GL_FLOAT, totalDataInSingleVertex * sizeof(GLfloat), BUFFER_OFFSET(12)); //The starting point of normals, 12 bytes away

	glDrawElements(GL_TRIANGLES, mesh.num_triangles * 3, GL_UNSIGNED_INT, 0);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();
}

void OpenglProject::setCamera(const Camera* camera)
{
	// apply camera transforms
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(camera->get_fov_degrees(), camera->get_aspect_ratio(), camera->get_near_clip(), camera->get_far_clip());
}

void OpenglProject::initLight() {
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

void OpenglProject::computeMeshNormals(MeshData& mesh) {
	// initialize previous normal
	for (unsigned int i = 0; i < mesh.num_vertices; i++) {
		mesh.normals[i] = { 0, 0, 0 };
	}

	// calculate normal based on triangle
	for (unsigned int i = 0; i < mesh.num_triangles; i++) {
		// normal vector = (b-a) x (c-a)
		Vector3 a = mesh.vertices[mesh.triangles[i].vertices[0]];
		Vector3 b = mesh.vertices[mesh.triangles[i].vertices[1]];
		Vector3 c = mesh.vertices[mesh.triangles[i].vertices[2]];

		Vector3 normal = normalize(cross((b - a), (c - a)));

		// assign the normal to the vertices
		for (unsigned int j = 0; j < 3; j++) {
			mesh.normals[mesh.triangles[i].vertices[j]] += normal;
		}
	}

	// normalize the normal
	for (unsigned int i = 0; i < mesh.num_vertices; i++) {
		mesh.normals[i] = normalize(mesh.normals[i]);
	}
}

void OpenglProject::generateWaterMesh(unsigned int resolution) {
	float range[2] = { -1, 1 };
	double x = range[0], z = range[0];
	double space = (range[1] - range[0]) / (double) resolution;

	// allocate memory for vertices
	this->scene.heightmap->mesh.num_vertices = std::pow(resolution + 1, 2);
	this->scene.heightmap->mesh.vertices = new Vector3[this->scene.heightmap->mesh.num_vertices];
	// allocate memory for triangles 2*(n^2)
	this->scene.heightmap->mesh.num_triangles = 2 * std::pow(resolution, 2);
	this->scene.heightmap->mesh.triangles = new Triangle[this->scene.heightmap->mesh.num_triangles];

	unsigned int vertexIndex = 0;
	unsigned int triangleIndex = 0;

	while (x <= range[1]) {
		// init z
		z = range[0];
		
		while (z <= range[1]) {
			// assign vertex
			this->scene.heightmap->mesh.vertices[vertexIndex].x = x;
			this->scene.heightmap->mesh.vertices[vertexIndex].z = z;
			this->scene.heightmap->mesh.vertices[vertexIndex].y = this->scene.heightmap->compute_height(Vector2(x, z));

			// assign triangle index
			if (z < range[1] && x < range[1]) { // don't add triangle to last row and column (we already add it)
				// index -- i1
				//     | \ 1|
				//     |2 \ |
				//    i2 -- i3
				unsigned int i1 = vertexIndex + 1;
				unsigned int i2 = vertexIndex + resolution + 1;
				unsigned int i3 = i2 + 1;

				// first triangle (1. top part)
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[0] = vertexIndex;
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[1] = i1;
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[2] = i3;
				triangleIndex++;
				// second triangle (2. bottom part)
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[0] = vertexIndex;
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[1] = i3;
				this->scene.heightmap->mesh.triangles[triangleIndex].vertices[2] = i2;
				triangleIndex++;
			}

			// increment z;
			z += space;
			vertexIndex++;
		}

		// increment x
		x += space;
	}

	// allocate memory for normal
	this->scene.heightmap->mesh.normals = new Vector3[this->scene.heightmap->mesh.num_vertices];
	// calculate normal for water
	computeMeshNormals(this->scene.heightmap->mesh);
}

} /* _462 */
