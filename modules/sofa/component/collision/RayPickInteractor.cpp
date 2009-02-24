/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/component/collision/RayPickInteractor.inl>
#include <sofa/component/collision/RayContact.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/collision/TetrahedronModel.h>
#include <sofa/component/collision/SphereTreeModel.h>
#include <sofa/component/collision/DistanceGridCollisionModel.h>
#include <sofa/component/mapping/BarycentricMapping.h>
#include <sofa/component/mapping/RigidMapping.h>
#include <sofa/component/mapping/SkinningMapping.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/helper/Factory.inl>
#include <sofa/helper/system/gl.h>


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;

template<>
void DefaultPickingManager<defaulttype::Vec3Types,forcefield::VectorSpringForceField<defaulttype::Vec3Types> >::addContact(forcefield::VectorSpringForceField<defaulttype::Vec3Types>* ff, int index1, int index2, double stiffness, double mu_v, double /*length*/, const Vector3& p1, const Vector3& p2)
{
    ff->addSpring(index1, index2, stiffness, mu_v, (p2-p1));
}

template<>
void DefaultPickingManager<defaulttype::Vec3Types,forcefield::StiffSpringForceField<defaulttype::Vec3Types> >::addContact(forcefield::StiffSpringForceField<defaulttype::Vec3Types>* ff, int index1, int index2, double stiffness, double mu_v, double length, const Vector3& /*p1*/, const Vector3& /*p2*/)
{
    ff->addSpring(index1, index2, stiffness, mu_v, length);
}
    
template class DefaultPickingManager< Vec3Types, forcefield::VectorSpringForceField<Vec3Types> >;

helper::Creator<BasePickingManager::PickingManagerFactory, DefaultPickingManager< Vec3Types, forcefield::VectorSpringForceField<Vec3Types> > > PickingVectorSpringVec3Class ("VectorSpringVec3d",true);


BasePickingManager* BasePickingManager::Create(std::string name, core::objectmodel::BaseContext* context)
{
    BasePickingManager* o = PickingManagerFactory::CreateObject(name, context);
    if (o != NULL)
    {
	o->setName(name);
	if (context != NULL)
	    context->addObject(o);
    }
    return o;
}

int BasePickingManager::CreateAll(std::vector<BasePickingManager*>& instances, core::objectmodel::BaseContext* context)
{
    int n = 0;
    for (PickingManagerFactory::iterator it = PickingManagerFactory::getInstance()->begin(), itend = PickingManagerFactory::getInstance()->end(); it != itend; ++it)
    {
	BasePickingManager* o = it->second->createInstance(context);
	if (o != NULL)
	{
	    o->setName(it->first);
	    if (context != NULL)
		context->addObject(o);
	    instances.push_back(o);
	    ++n;
	}
    }
    return n;
}


BasePickingManager::~BasePickingManager()
{
}



RayPickInteractor::RayPickInteractor()
:  state(DISABLED), button(0)
{
    transform.identity();
    this->contactStiffness.setValue(100);
}

RayPickInteractor::~RayPickInteractor()
{
}

void RayPickInteractor::init()
{
    this->RayModel::init();
    this->getContext()->get<BasePickingManager,helper::vector<BasePickingManager*> >(&pickManagers,core::objectmodel::BaseContext::SearchRoot);
    if (pickManagers.empty())
    {
	std::cout << "RayPickInteractor: creating default pick managers." << std::endl;
	BasePickingManager::CreateAll(pickManagers, this->getContext());
    }
}


bool RayPickInteractor::isActive()
{
    return state==ACTIVE || state==PRESENT || state == FIRST_INPUT || state == IS_CUT; // state FIRST_INPUT controls the first input point for incision in a triangular mesh
}

bool RayPickInteractor::isAttached()
{
    for (unsigned int i=0; i<pickManagers.size(); i++)
	if (pickManagers[i]->isAttached())
	    return true;
    return false;
}

void RayPickInteractor::pickBody()
{
	const int oldState = state;

	// We find the first contact for each ray
	for (int r=0; r<getNbRay(); r++)
	{
		Ray ray = getRay(r);
		double dist = 0;
		core::componentmodel::collision::DetectionOutput* collision = findFirstCollision(ray,&dist);
		if (collision==NULL)
			continue;
		core::CollisionElementIterator elem1 = collision->elem.first;
		core::CollisionElementIterator elem2 = collision->elem.second;
		Vector3 p1 = collision->point[0];
		Vector3 p2 = collision->point[1];
		if (elem2.getCollisionModel() == this)
		{ // swap elements
			elem2 = collision->elem.first;
			elem1 = collision->elem.second;
			p2 = collision->point[0];
			p1 = collision->point[1];
		}
		bool processed = false;
		for (unsigned int i=0;!processed && i<pickManagers.size();++i)
		{
		    if (pickManagers[i]->attach(button, elem1, elem2, dist, p1, p2))
			processed = true;
		}
		if (!processed) // default cases not yet handled by external managers
		switch(button)
		{
		case 0:	// attach body
		break;

		case 1: // element removal
			{

				sofa::core::componentmodel::topology::TopologyModifier* topoMod; 
				elem2.getCollisionModel()->getContext()->get(topoMod);	

				if(topoMod){ // dynamic topology
					// Handle Removing of topological element (from any type of topology)
					topo_changeManager.removeItemsFromCollisionModel(elem2);
				}
			}
			break;

		case 2: // incision
			{
				sofa::core::componentmodel::topology::TopologyModifier* topoMod; 
				elem2.getCollisionModel()->getContext()->get(topoMod);	

				if(topoMod){ // dynamic topology
					// Handle Cutting, using global variables to register the two last input points
					bool attach = topo_changeManager.incisionCollisionModel(elem2, p2, state == FIRST_INPUT, state == IS_CUT);
					if(attach)
						state = ATTACHED;
				}
			}
			break;
		}
	}

	if(oldState == FIRST_INPUT || oldState == IS_CUT)
	{
		state = SECOND_INPUT; // force the state SECOND_INPUT
	}
	else if (isAttached())
	{
		state = ATTACHED;
	}

	if (oldState != state)
	{
		std::cout << "Interactor State: ";
		switch (state)
		{
		case DISABLED:
			std::cout << "DISABLED";
			break;
		case PRESENT:
			std::cout << "PRESENT";
			break;
		case ACTIVE:
			std::cout << "ACTIVE";
			break;
		case ATTACHED:
			std::cout << "ATTACHED";
			break;
		case FIRST_INPUT:
			std::cout << "FIRST_INPUT"; // first input point for incision in a triangular mesh
			break;
		case SECOND_INPUT:
			std::cout << "SECOND_INPUT"; // second input point for incision in a triangular mesh
			break;
		case IS_CUT:
			std::cout << "IS_CUT"; 
			break;
		}
		std::cout << std::endl;
	}
}

void RayPickInteractor::releaseBody()
{
    for (unsigned int i=0; i<pickManagers.size(); i++)
	if (pickManagers[i]->isAttached())
	    pickManagers[i]->release();
}

/// Computation of a new simulation step.
void RayPickInteractor::updatePosition(double /*dt*/)
{
	if (state == DISABLED && getNbRay()>0)
	{ // we need to disable our interactor
		resize(0);
	}
	else if (state != DISABLED && getNbRay()==0)
	{ // we need to enable our interactor
		std::cout << "ADD RAY"<<std::endl;
		setNbRay(1);
		getRay(0).l() = 10000;
	}

	if ((state == ACTIVE || state == FIRST_INPUT || state == IS_CUT) && !contacts.empty()) // state FIRST_INPUT controls the first input point for incision in a triangular mesh
	{ 
		pickBody();
	} 
	else if (state != ATTACHED && isAttached())
	{ 
		releaseBody();
	}

	// update the ray position
	if (getNbRay()>=1)
	{ // update current ray position
		Ray ray = getRay(0);
		ray.origin() = transform*Vec4d(0,0,0,1);
		ray.direction() = transform*Vec4d(0,0,1,0);
		ray.direction().normalize();
	}

    for (unsigned int i=0; i<pickManagers.size(); i++)
	pickManagers[i]->update();
}

/// Interactor interface
void RayPickInteractor::newPosition(const Vector3& /*translation*/, const Quat& /*rotation*/, const Mat4x4d& transformation)
{
    transform = transformation;
}

void RayPickInteractor::newEvent(const std::string& command)
{
    const int oldState = state;

	if (command == "hide")
    {
        state = DISABLED;
	}
	else
	{
		switch(oldState)
		{
		case DISABLED:
			if (command == "show")
			{
				state = PRESENT;
			}
		case PRESENT:
			if (command == "pick" || command == "pick1") // detection of Left-click 
			{
				state = ACTIVE;
				button = 0;
			}
			else if (command == "pick2") // detection of Right-click
			{
				state = ACTIVE;
				button = 1;
			}
			break;
		case ACTIVE:
		case ATTACHED:
			if (command == "release")
				state = PRESENT;
			break;
		case FIRST_INPUT:
			// this state changes into SECOND_INPUT after a position update
			break;
		case IS_CUT:
			// this state changes into SECOND_INPUT after a position update
			break;
		case SECOND_INPUT:
			if (command == "pick3") // detection of Mid-click
			{
				state = IS_CUT;
				button = 2;
			}
			break;
		}
	}

	if (command == "pick3") // detection of Mid-click
	{
		if (oldState!=SECOND_INPUT)
		{
			state = FIRST_INPUT;
			button = 2;
		}
	}

    if (oldState != state)
    {
        std::cout << "Interactor State: ";
        switch (state)
        {
        case DISABLED:
            std::cout << "DISABLED";
            break;
        case PRESENT:
            std::cout << "PRESENT";
            break;
        case ACTIVE:
            std::cout << "ACTIVE, button = "<<button;
            break;
        case ATTACHED:
            std::cout << "ATTACHED";
            break;
		case FIRST_INPUT:
            std::cout << "FIRST_INPUT, button = "<<button; // First input point for incision
            break;
        case SECOND_INPUT:
            std::cout << "SECOND_INPUT, button = "<<button; // Second input point for incision
            break;
		case IS_CUT:
            std::cout << "IS_CUT";
            break;
        }
        std::cout << std::endl;
    }
}

core::componentmodel::collision::DetectionOutput* RayPickInteractor::findFirstCollision(const Ray& ray, double* dist)
{
    core::componentmodel::collision::DetectionOutput* result = NULL;
    const Vector3& origin = ray.origin();
    const Vector3& direction = ray.direction();
    double l = ray.l();
    double mindist = 0;
    for (std::set<BaseRayContact*>::iterator it = contacts.begin(); it!=contacts.end(); ++it)
    {
        const sofa::helper::vector<core::componentmodel::collision::DetectionOutput*>& collisions = (*it)->getDetectionOutputs();
        for (unsigned int i=0;i<collisions.size();i++)
        {
            core::componentmodel::collision::DetectionOutput* collision = collisions[i];
            if (collision->elem.second == ray)
            {
                if (!collision->elem.first.getCollisionModel()->isSimulated())
                    continue;
                double d = (collision->point[1] - origin)*direction;
                if (d<0.0 || d>l)
                    continue;
                if (result==NULL || d<mindist)
                {
                    result = collision;
                    mindist = d;
                }
            }
            else
                if (collision->elem.first == ray)
                {
                    if (!collision->elem.second.getCollisionModel()->isSimulated())
                        continue;
                    double d = (collision->point[0] - origin)*direction;
                    if (d<0.0 || d>l)
                        continue;
                    if (result==NULL || d<mindist)
                    {
                        result = collision;
                        mindist = d;
                    }
                }
        }
    }
    if (dist!=NULL && result!=NULL)
        *dist = mindist;
    return result;
}

void RayPickInteractor::draw()
{
    if (getNbRay()>=1)
        this->RayModel::draw();
    if (state==PRESENT)
        glColor4f(0,1,0,1);
    else if (state==ATTACHED)
        glColor4f(1,1,1,1);
    else
        return;
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(3);
    for (int r = 0; r < getNbRay(); r++)
    {
        Ray ray = getRay(r);
        double dist = 0;
        core::componentmodel::collision::DetectionOutput* collision = findFirstCollision(ray,&dist);
        if (collision==NULL)
            continue;
        core::CollisionElementIterator elem = collision->elem.first;
        if (elem.getIndex() < elem.getCollisionModel()->getSize())
            elem.draw();
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4f(1,1,1,1);
    glLineWidth(1);
}

} // namespace collision

} // namespace component

} // namespace sofa
