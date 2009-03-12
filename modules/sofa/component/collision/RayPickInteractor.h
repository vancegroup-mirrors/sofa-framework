/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_H
#define SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_H

#include <sofa/component/collision/RayModel.h>
#include <sofa/core/BehaviorModel.h>

#include <sofa/component/container/MechanicalObject.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/component/collision/BarycentricContactMapper.h>
#include <sofa/component/mapping/VoidMapping.h>


#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>

#include <sofa/component/collision/TopologicalChangeManager.h>

#include <sofa/helper/Factory.h>

namespace sofa
{
namespace component
{
namespace collision
{
using namespace sofa::defaulttype;

struct PickPoints
{
    core::CollisionModel* model1;
    int elem1;
    core::CollisionModel* model2;
    core::componentmodel::behavior::BaseMechanicalState* mstate2;
    int elem2;
    double dist;
    Vector3 p1, p2;
    PickPoints() : model1(NULL), elem1(0), model2(NULL), mstate2(NULL), elem2(0), dist(0) {}
};

class SOFA_COMPONENT_COLLISION_API BasePickingManager : public virtual sofa::core::objectmodel::BaseObject
{
public:

    virtual ~BasePickingManager();
    virtual bool attach(int /*button*/, core::CollisionElementIterator /*elem1*/, core::CollisionElementIterator /*elem2*/, double /*dist*/, Vector3 /*p1*/, Vector3 /*p2*/)
    {
	return false;
    }
    virtual bool attach(int button, PickPoints& a)
    {
	if (a.model1 && a.model2)
	    return attach(button, core::CollisionElementIterator(a.model1, a.elem1), core::CollisionElementIterator(a.model2, a.elem2), a.dist, a.p1, a.p2);
	else
	    return false;
    }
    virtual bool isAttached() = 0;
    virtual void release() = 0;
    virtual void update() = 0;

    typedef helper::Factory<std::string, BasePickingManager, core::objectmodel::BaseContext*> PickingManagerFactory;

    static BasePickingManager* Create(std::string name, core::objectmodel::BaseContext* context);
    static int CreateAll(std::vector<BasePickingManager*>& instances, core::objectmodel::BaseContext* context);

    template <class RealObject>
    static bool canCreate( RealObject*& /*obj*/, core::objectmodel::BaseContext* /*context*/)
    {
	return true;
    }

    template <class RealObject>
    static void create( RealObject*& obj, core::objectmodel::BaseContext* context)
    {
	if (RealObject::canCreate(obj, context))
	{
	    obj = new RealObject;
	    //if (context != NULL)
	    //    context->addObject(obj);
	}
    }
};

class SOFA_COMPONENT_COLLISION_API RayPickInteractor : public RayModel, public core::BehaviorModel, public mapping::VoidMapping
{
public:
	RayPickInteractor();
	~RayPickInteractor();

	virtual void init();

	virtual bool isActive();

    virtual bool pickParticles(double /*rayOx*/, double /*rayOy*/, double /*rayOz*/, double /*rayDx*/, double /*rayDy*/, double /*rayDz*/, double /*radius0*/, double /*dRadius*/,
			       std::multimap< double, std::pair<sofa::core::componentmodel::behavior::BaseMechanicalState*, int> >& /*particles*/)
    {
	return false;
    }

	/// Computation of a new simulation step.
	virtual void updatePosition(double dt);

	/// Interactor interface
	virtual void newPosition(const Vector3& translation, const Quat& rotation, const Mat4x4d& transformation);
	virtual void newEvent(const std::string& command);
	/// Set ray radius, at positions (0,0,0) and (0,0,1) in last transmitted transformation
	virtual void setRayRadius(double r0, double r1);

	virtual void draw();

    virtual bool isAttached();

    Data<bool> useCollisions;

protected:
	void pickBody();
	void releaseBody();

	bool findFirstCollision(const Ray& ray, PickPoints& result, bool create = false);

	// TODO: remove the FIRST_INPUT, SECOND_INPUT, IS_CUT states
    enum { DISABLED, PRESENT, ACTIVE, ATTACHED, FIRST_INPUT, SECOND_INPUT, IS_CUT } state; // FIRST_INPUT and SECOND_INPUT are states to control inputs points for incision in a triangular mesh
	int button; // index of activated button (only valid in ACTIVE state)
	Mat4x4d transform;
	Real rayRadius0, rayRadius1;

    helper::vector<BasePickingManager*> pickManagers;

    helper::vector<PickPoints*> pickPoints;

	// TopologicalChangeManager object to handle topological changes
	TopologicalChangeManager topo_changeManager;

};


template<class TDataTypes, class ContactForceField>
class DefaultPickingManager : public BasePickingManager
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;

    Data<int> button;

    DefaultPickingManager();

    virtual bool attach(int button, PickPoints& a)
    {
        if (button == this->button.getValue())
	{
	    if (a.model1 && a.model2)
		return attach(core::CollisionElementIterator(a.model1, a.elem1), core::CollisionElementIterator(a.model2, a.elem2), a.dist, a.p1, a.p2);
	    else if (a.model1 && a.mstate2)
		return attach(core::CollisionElementIterator(a.model1, a.elem1), a.mstate2, a.elem2, a.dist, a.p1, a.p2);
	}
	return false;
    }
    virtual bool isAttached()
    {
        return !attachedPoints.empty();
    }
    virtual void release();
    virtual void update();
    virtual void draw();

    template <class RealObject>
    static bool canCreate( RealObject*& obj, core::objectmodel::BaseContext* context)
    {
	if (context != NULL)
	{
	    core::componentmodel::behavior::MechanicalState<DataTypes>* p = NULL;
	    context->get(p, core::objectmodel::BaseContext::SearchRoot);
	    if (p == NULL)
		return false;
	}
	return BasePickingManager::canCreate(obj, context);
    }

protected:

    virtual component::MechanicalObject<DataTypes>* getMState(core::objectmodel::BaseContext* context);

    virtual bool attach(core::CollisionElementIterator elem1, core::CollisionElementIterator elem2, double dist, Vector3 p1, Vector3 p2);

    virtual bool attach(core::CollisionElementIterator elem1, core::componentmodel::behavior::BaseMechanicalState* mstate2, int elem2, double dist, Vector3 p1, Vector3 p2);

    virtual void addContact(ContactForceField* ff, int index1, int index2, double stiffness, double mu_v, double length, const Vector3& p1, const Vector3& p2);

    sofa::helper::vector< std::pair<Ray,double> > attachedPoints;
    component::MechanicalObject<DataTypes>* mechanicalObject;
    sofa::helper::vector<core::componentmodel::behavior::BaseForceField*> forcefields;
    sofa::helper::vector<BaseContactMapper<DataTypes>*> mappers;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
