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

#include <sofa/component/collision/AttachBodyPerformer.h>
#include <sofa/component/collision/MouseInteractor.h>

namespace sofa
{

namespace component
{

namespace collision
{

template <class DataTypes>
void AttachBodyPerformer<DataTypes>::start()
{
    BodyPicked picked=this->interactor->getBodyPicked();
    if (!picked.body && !picked.mstate) return;
    core::componentmodel::behavior::MechanicalState<DataTypes>* mstateCollision=NULL;
    int index;
    double restLength=0;
    if (picked.body)
    {
        mapper = MouseContactMapper::Create(picked.body);

        if (!mapper)
        {
            std::cerr << "Problem with Mouse Mapper creation : " << std::endl;
            return;
        }
        std::string name = "contactMouse";
        mstateCollision = mapper->createMapping(name.c_str());
        mapper->resize(1);

        const typename DataTypes::Coord pointPicked=picked.point;
        const int idx=picked.indexCollisionElement;
        typename DataTypes::Real r=0.0;

        index = mapper->addPoint(pointPicked, idx, r);
        mapper->update();
    }
    else
    {
        mstateCollision = dynamic_cast< core::componentmodel::behavior::MechanicalState<DataTypes>*  >(picked.mstate);
        index = picked.indexCollisionElement;
        if (!mstateCollision)
        {
            this->interactor->serr << "uncompatible MState during Mouse Interaction " << this->interactor->sendl;
            return;
        }
    }

    double distanceFromMouse=picked.rayLength;
    restLength=picked.dist;
    this->interactor->setDistanceFromMouse(picked.rayLength);
    const double friction=0.0;

    forcefield = new MouseForceField(dynamic_cast<MouseContainer*>(this->interactor->getMouseContainer()), mstateCollision); forcefield->setName("Spring-Mouse-Contact");

    forcefield->addSpring(0,index, stiffness, friction, restLength);

    this->interactor->getMouseRayModel()->getRay(0).origin() += this->interactor->getMouseRayModel()->getRay(0).direction()*distanceFromMouse;

    sofa::core::BaseMapping *mapping;
    this->interactor->getContext()->get(mapping); assert(mapping);
    mapping->updateMapping();

    mstateCollision->getContext()->addObject(forcefield);
    forcefield->init();
    this->interactor->setMouseAttached(true);
}

template <class DataTypes>
void AttachBodyPerformer<DataTypes>::execute()
{
};

template <class DataTypes>
void AttachBodyPerformer<DataTypes>::draw()
{
    if (forcefield)
    {
        bool b = forcefield->getContext()->getShowInteractionForceFields();
        forcefield->getContext()->setShowInteractionForceFields(true);
        forcefield->draw();
        forcefield->getContext()->setShowInteractionForceFields(b);
    }
}

template <class DataTypes>
AttachBodyPerformer<DataTypes>::AttachBodyPerformer(BaseMouseInteractor *i):TInteractionPerformer<DataTypes>(i), mapper(NULL),forcefield(NULL)
{
}


template <class DataTypes>
AttachBodyPerformer<DataTypes>::~AttachBodyPerformer()
{
    if (forcefield)
    {
        forcefield->cleanup();
        forcefield->getContext()->removeObject(forcefield);
        delete forcefield; forcefield=NULL;
    }

    if (mapper)
    {
        mapper->cleanup();
        delete mapper; mapper=NULL;
    }

    this->interactor->setDistanceFromMouse(0);
    this->interactor->setMouseAttached(false);
};


#ifdef WIN32
helper::Creator<InteractionPerformer::InteractionPerformerFactory, AttachBodyPerformer<defaulttype::Vec3Types> >  AttachBodyPerformerVec3Class("AttachBody");
#endif

}
}
}
