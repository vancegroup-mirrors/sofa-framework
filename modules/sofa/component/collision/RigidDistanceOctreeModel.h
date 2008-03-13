#ifndef SOFA_COMPONENT_COLLISION_RIGIDDISTANCEOCTREEMODEL_H
#define SOFA_COMPONENT_COLLISION_RIGIDDISTANCEOCTREEMODEL_H

#include <sofa/core/CollisionModel.h>
#include <sofa/core/VisualModel.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include "SLC/slcSurface.H"


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;

class RigidDistanceOctreeModel;

class RigidDistanceOctree : public core::CollisionElement
{
protected:
	slcSurface* surface;
	int index;
	RigidDistanceOctreeModel* model;
public:

	RigidDistanceOctree(int idx, RigidDistanceOctreeModel* model)
	: surface(NULL), index(idx), model(model)
	{
	}
	~RigidDistanceOctree()
	{
		if (surface!=NULL) delete surface;
	}
	
	void getBBox (Vector3 &minBBox, Vector3 &maxBBox);
	void getBBox (double* minBBox, double* maxBBox);
	bool isSelfCollis (CollisionElement *elem);
	void clear();
	
	slcSurface* getSurface() { return surface; }
	
	void setSurface(slcSurface* surf) { if (surface!=NULL && surface!=surf) delete surface; surface = surf; }
	
	void draw();
	
	int getIndex() { return index; }
	
	const Vector3& getPosition() { return center(); }
	
//	void addForce (const Vector3 &force);
	
	core::CollisionModel* getCollisionModel() { return model; }
	
	friend class RigidDistanceOctreeModel;
};

//class : public 
//{
//};

class RigidDistanceOctreeModel : public component::MechanicalObject<RigidTypes>, public core::CollisionModel, public core::VisualModel
{
protected:
	sofa::helper::vector<core::CollisionElement*> elems;
	core::CollisionModel* previous;
	core::CollisionModel* next;
	core::BehaviorModel* object;
	
	class Loader;
	void init(const char* filename);
	
	VecCoord* internalForces;
	VecCoord* externalForces;
public:
	
	RigidDistanceOctreeModel(const char* filename, const std::string& name);
	
	// -- MechanicalState interface
	
	virtual void setObject(core::BehaviorModel* obj);
	
	virtual void beginIteration(double dt);
	
	virtual void endIteration(double dt);
	
	virtual void accumulateForce();
	
	// -- CollisionModel interface
	
	virtual core::BehaviorModel* getObject()
	{ return object; }
	
	void computeBoundingBox();
	
	sofa::helper::vector<core::CollisionElement*> & getCollisionElements()
	{ return elems; }
	
	core::CollisionModel* getNext()
	{ return next; }
	
	core::CollisionModel* getPrevious()
	{ return previous; }
	
	void setNext(core::CollisionModel* n)
	{ next = n; }
	
	void setPrevious(core::CollisionModel* p)
	{ previous = p; }
	
	void applyTranslation(double dx, double dy, double dz);
	
	// -- VisualModel interface
	
	void draw();
	
	void initTextures() { }
	
	void update() { }
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
