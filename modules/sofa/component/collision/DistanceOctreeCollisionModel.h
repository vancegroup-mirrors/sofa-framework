
#ifndef SOFA_COMPONENT_COLLISION_DISTANCEOCTREECOLLISIONMODEL_H
#define SOFA_COMPONENT_COLLISION_DISTANCEOCTREECOLLISIONMODEL_H

#include <sofa/core/CollisionModel.h>
#include <sofa/core/VisualModel.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/component/topology/MeshTopology.h>
#include <SLC/slcSurface.h>


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace SLC;

class DistanceOctreeCollisionModel;

class DistanceOctreeCollisionElement : public core::TCollisionElementIterator<DistanceOctreeCollisionModel>
{
public:
	DistanceOctreeCollisionElement(DistanceOctreeCollisionModel* model, int index);

	explicit DistanceOctreeCollisionElement(core::CollisionElementIterator& i);
	
	SlcSurface* getSurface();
	
	void setSurface(SlcSurface* surf);
};

class DistanceOctreeCollisionModel : public core::CollisionModel, public core::VisualModel
{
protected:
	sofa::helper::vector<SlcSurface*> elems;
	core::CollisionModel* previous;
	core::CollisionModel* next;
	bool static_;

	// Input data parameters
	std::string filename;
	Vec3d translation;
	double scale;

	// Output tree parameters
	double border;
	int depth;
	bool marchingcube;
	std::string dumpfilename;

	core::componentmodel::behavior::MechanicalState<Vec3Types>* mstate;
	core::componentmodel::behavior::MechanicalState<RigidTypes>* rigid;
	topology::MeshTopology* mesh;
	
	void updateOctree();
public:

    DistanceOctreeCollisionModel();

    void parse(core::objectmodel::BaseObjectDescription* arg);
	
	~DistanceOctreeCollisionModel();
	
	const std::string& getFilename() const   { return filename; }
	void setFilename(const std::string& val) { filename = val;  }
	
	const Vec3d& getTranslation() const   { return translation; }
	void setTranslation(const Vec3d& val) { translation = val;  }

	const double& getScale() const   { return scale; }
	void setScale(const double& val) { scale = val;  }

	const double& getBorder() const   { return border; }
	void setBorder(const double& val) { border = val;  }

	const int& getDepth() const   { return depth; }
	void setDepth(const int& val) { depth = val;  }

	core::componentmodel::behavior::MechanicalState<Vec3Types>* getMechanicalState() { return mstate; }
	core::componentmodel::behavior::MechanicalState<RigidTypes>* getRigidModel() { return rigid; }

	void init();
	
	SlcSurface* getSurface(int index=0);
	
	void setSurface(SlcSurface* surf, int index=0);

	// -- CollisionModel interface

	void resize(int size);

	/// Create or update the bounding volume hierarchy.
	void computeBoundingTree(int maxDepth=0);
	
	void draw(int index);
	
	// -- VisualModel interface
	
	void draw();
	
	void initTextures() { }
	
	void update() { }
};

inline DistanceOctreeCollisionElement::DistanceOctreeCollisionElement(DistanceOctreeCollisionModel* model, int index)
: core::TCollisionElementIterator<DistanceOctreeCollisionModel>(model, index)
{}

inline DistanceOctreeCollisionElement::DistanceOctreeCollisionElement(core::CollisionElementIterator& i)
: core::TCollisionElementIterator<DistanceOctreeCollisionModel>(static_cast<DistanceOctreeCollisionModel*>(i.getCollisionModel()), i.getIndex())
{
}

inline SlcSurface* DistanceOctreeCollisionElement::getSurface() { return model->getSurface(index); }
inline void DistanceOctreeCollisionElement::setSurface(SlcSurface* surf) { return model->setSurface(surf, index); }

} // namespace collision

} // namespace component

} // namespace sofa

#endif
