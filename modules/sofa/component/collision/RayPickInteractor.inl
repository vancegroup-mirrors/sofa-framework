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
#ifndef SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_INL
#define SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_INL

#include <sofa/component/collision/RayPickInteractor.h>
#include <sofa/component/collision/RayContact.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/collision/TetrahedronModel.h>
#include <sofa/component/collision/SphereTreeModel.h>
#include <sofa/component/collision/DistanceGridCollisionModel.h>
#include <sofa/component/mapping/BarycentricMapping.h>
#include <sofa/component/mapping/RigidMapping.h>
#include <sofa/component/mapping/SkinningMapping.h>
#include <sofa/component/forcefield/StiffSpringForceField.h>
#include <sofa/component/forcefield/VectorSpringForceField.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>

#include <sofa/helper/system/gl.h>

namespace sofa
{

namespace component
{

namespace collision
{

typedef mapping::SkinningMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<defaulttype::Rigid3Types>, core::componentmodel::behavior::MechanicalState<defaulttype::Vec3Types> > > MySkinningMapping;

using namespace sofa::defaulttype;

template<class DataTypes, class ContactForceField>
DefaultPickingManager<DataTypes,ContactForceField>::DefaultPickingManager()
  : button( initData( &button, 0, "button", "mouse button which this manager responds to (0 = Left, 1 = Right)" ) ) //", 2 = Middle" has been removed as, in the current state, middle is hard coded for cutting
, mechanicalObject(NULL)
{
}

template<class DataTypes, class ContactForceField>
void DefaultPickingManager<DataTypes,ContactForceField>::release()
{
    // release the attached body
    for (unsigned int i=0; i<forcefields.size(); i++)
    {
	sout << "DefaultPickingManager: Deleting "<<forcefields[i]->getClassName()<<" "<<forcefields[i]->getName()<<sendl;
	forcefields[i]->cleanup();
	forcefields[i]->getContext()->removeObject(forcefields[i]);
	delete forcefields[i];
    }
    forcefields.clear();
    for (unsigned int i=0;i<mappers.size();i++)
    {
	mappers[i]->cleanup();
        delete mappers[i];
    }
    mappers.clear();
    attachedPoints.clear();
}

template<class DataTypes, class ContactForceField>
void DefaultPickingManager<DataTypes,ContactForceField>::update()
{
    if (mechanicalObject!=NULL && !attachedPoints.empty())
    {
	typename DataTypes::VecCoord& x = *mechanicalObject->getX();
	for (unsigned int i=0;i<attachedPoints.size();i++)
	{
	    Ray& ray = attachedPoints[i].first;
	    x[i] = ray.origin()+ray.direction()*attachedPoints[i].second;
	}
    }
}

template<class DataTypes, class ContactForceField>
void DefaultPickingManager<DataTypes,ContactForceField>::draw()
{
    if (!forcefields.empty())
    {
	glColor4f(1,1,1,1);
	glLineWidth(3);
	for (unsigned int i=0;i<forcefields.size();i++)
	{
	    if (forcefields[i] != NULL && forcefields[i]->getContext() != NULL)
	    {
		// Hack to make forcefields visible
		bool b = forcefields[i]->getContext()->getShowInteractionForceFields();
		forcefields[i]->getContext()->setShowInteractionForceFields(true);
		forcefields[i]->draw();
		forcefields[i]->getContext()->setShowInteractionForceFields(b);
	    }
	}
	glLineWidth(1);
    }
}

template<class DataTypes, class ContactForceField>
component::MechanicalObject<DataTypes>* DefaultPickingManager<DataTypes,ContactForceField>::getMState(core::objectmodel::BaseContext* context)
{
    if (mechanicalObject == NULL)
    {
        mechanicalObject = new component::MechanicalObject<DataTypes>();
        simulation::Node* parent = dynamic_cast<simulation::Node*>(context);
        if (parent != NULL)
        {
            std::string name = "contactMouse";
            name += DataTypes::Name();
            simulation::Node* child = simulation::getSimulation()->newNode(name);
            parent->addChild(child);
            child->updateContext();
            child->addObject(mechanicalObject);
	    if (parent->get<sofa::core::componentmodel::behavior::OdeSolver>()) // we are below a solver, use a VoidMapping to indicate that it should not touch this object
		child->addObject(new component::mapping::VoidMapping);
        }
        else
        {
            context->addObject(mechanicalObject);
            //mechanicalObject->setContext(context);
        }
        mechanicalObject->init();
    }
    return mechanicalObject;
}

template<class DataTypes, class ContactForceField>
bool DefaultPickingManager<DataTypes,ContactForceField>::attach(core::CollisionElementIterator elem1, core::CollisionElementIterator elem2, double dist, Vector3 p1, Vector3 p2)
{
    core::CollisionModel* model1 = elem1.getCollisionModel();
    core::CollisionModel* model2 = elem2.getCollisionModel();
    RayModel* rayModel = dynamic_cast<RayModel*>(model1);
    if (rayModel == NULL) return false;
    BaseContactMapper<DataTypes>* mapper2 = BaseContactMapper<DataTypes>::Create(model2);
    if (mapper2 == NULL)
    {
        serr << "DefaultPickingManager: ContactMapper from "<<model2->getClassName()<<"<"<<model2->getTemplateName()<<"> to "<<DataTypes::Name()<<" not found."<<sendl;
	//sofa::helper::printFactoryLog();
        return false;
    }
    component::MechanicalObject<DataTypes>* mstate1 = getMState(rayModel->getContext());
    std::string name = "contactMouse";
    name += DataTypes::Name();
    MechanicalState<DataTypes>* mstate2 = mapper2->createMapping(name.c_str());
    ContactForceField* ff = new ContactForceField(mstate1, mstate2);
    name = this->getName();
    name += "-";
    name += model2->getClassName();
    ff->setName(name);
    ff->clear(1);
    mapper2->resize(1);
    int index1 = attachedPoints.size();
    mstate1->resize(index1+1);
    int index2 = elem2.getIndex();
    Real r1 = 0.0;
    Real r2 = 0.0;
    // Create mapping for second point
    Coord p; p = p2;
    index2 = mapper2->addPoint(p, index2, r2);
    mapper2->update();
    p2 = (*mstate2->getX())[index2];
    double stiffness = (elem1.getContactStiffness() * elem2.getContactStiffness());
    double mu_v = (elem1.getContactFriction() + elem2.getContactFriction());
    addContact(ff, index1, index2, stiffness, mu_v, r1+r2, p1, p2);
    mstate2->getContext()->addObject(ff);
    ff->init();
    forcefields.push_back(ff);
    mappers.push_back(mapper2);
    attachedPoints.push_back(std::make_pair(Ray(rayModel,elem1.getIndex()), dist));
    return true;
}

template<class DataTypes, class ContactForceField>
bool DefaultPickingManager<DataTypes,ContactForceField>::attach(core::CollisionElementIterator elem1, core::componentmodel::behavior::BaseMechanicalState* model2, int elem2, double dist, Vector3 p1, Vector3 p2)
{
    core::CollisionModel* model1 = elem1.getCollisionModel();
    RayModel* rayModel = dynamic_cast<RayModel*>(model1);
    if (rayModel == NULL) return false;
    MechanicalState<DataTypes>* mstate2 = dynamic_cast<MechanicalState<DataTypes>*>(model2);
    if (mstate2 == NULL)
    {
        serr << "DefaultPickingManager: MechanicalState "<<model2->getClassName()<<"<"<<model2->getTemplateName()<<"> not supported."<<sendl;
        return false;
    }
    component::MechanicalObject<DataTypes>* mstate1 = getMState(rayModel->getContext());
    std::string name = "contactMouse";
    name += DataTypes::Name();
    ContactForceField* ff = new ContactForceField(mstate1, mstate2);
    name = this->getName();
    name += "-";
    name += model2->getClassName();
    ff->setName(name);
    ff->clear(1);
    int index1 = attachedPoints.size();
    mstate1->resize(index1+1);
    int index2 = elem2;
    Real r1 = 0.0;
    Real r2 = 0.0;
    // Create mapping for second point
    Coord p; p = p2;
    p2 = (*mstate2->getX())[index2];
    double stiffness = (elem1.getContactStiffness());
    double mu_v = (elem1.getContactFriction());
    addContact(ff, index1, index2, stiffness, mu_v, r1+r2, p1, p2);
    mstate2->getContext()->addObject(ff);
    ff->init();
    forcefields.push_back(ff);
    attachedPoints.push_back(std::make_pair(Ray(rayModel,elem1.getIndex()), dist));
    return true;
}


template<>
void DefaultPickingManager<defaulttype::Vec3Types,forcefield::VectorSpringForceField<defaulttype::Vec3Types> >::addContact(forcefield::VectorSpringForceField<defaulttype::Vec3Types>* ff, int index1, int index2, double stiffness, double mu_v, double /*length*/, const Vector3& p1, const Vector3& p2);

template<>
void DefaultPickingManager<defaulttype::Vec3Types,forcefield::StiffSpringForceField<defaulttype::Vec3Types> >::addContact(forcefield::StiffSpringForceField<defaulttype::Vec3Types>* ff, int index1, int index2, double stiffness, double mu_v, double length, const Vector3& /*p1*/, const Vector3& /*p2*/);

} // namespace collision

} // namespace component

} // namespace sofa

#endif
