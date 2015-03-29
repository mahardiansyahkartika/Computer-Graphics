/**
 * @file model.hpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_MODEL_HPP_
#define _462_SCENE_MODEL_HPP_

#include "scene/scene.hpp"
#include "scene/mesh.hpp"
#include "scene/meshtree.hpp"

namespace _462 {

/**
 * A mesh of triangles.
 */
class Model : public Geometry
{
public:

    const Mesh* mesh;
    const MeshTree *tree;
    const Material* material;

    Model();
    virtual ~Model();

    virtual void render() const;
    virtual bool initialize();

	// additional functions
	Intersection* getIntersection(Ray& r);
	void processHit(Intersection* hit);
	Bound createBoundingBox();
	void update(real_t delta_time);
	void calculateTriangleMiddlePoint();
};


} /* _462 */

#endif /* _462_SCENE_MODEL_HPP_ */

