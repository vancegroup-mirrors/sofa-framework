/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CONTRIB_TESTING_SENSABLE_H
#define SOFA_CONTRIB_TESTING_SENSABLE_H

#include <iostream>

#include <sofa/core/BehaviorModel.h>
#include <sofa/core/VisualModel.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/Quater.h>

using std::cout;

class SensAble : public sofa::core::VisualModel, public sofa::core::BehaviorModel
{

public:

	SensAble();
	~SensAble();

	// -- VisualModel interface
	void draw();
	void initTextures() { }
	void update() { }

    
    Data<sofa::defaulttype::Vec3d> translation;
	Data<sofa::defaulttype::Quat> rotation;
	Data<double> scale;
	Data<double> tipOffset;
	Data<sofa::defaulttype::RigidTypes::Coord> xform;
	Data<sofa::defaulttype::RigidTypes::Coord> xform2;
	Data<sofa::defaulttype::Vec3d> pivot;
    Data<bool> flip;
    Data<bool> bDraw;
    Data<char> key1; bool button1State;
    Data<char> key2; bool button2State;

	sofa::defaulttype::Mat3x3d mrotation;

	sofa::component::collision::SphereModel* sphereModel;
	sofa::component::MechanicalObject<sofa::defaulttype::RigidTypes>* rigidModel;

	// -- CollisionModel interface
/*
	virtual std::vector<CollisionElement*> & getCollisionElements();
	virtual CollisionModel* getNext() { return sphereModel->getNext(); }
	virtual CollisionModel* getPrevious() { return sphereModel->getPrevious(); }
	virtual BehaviorModel* getObject() { return sphereModel->getObject(); }
*/
	// -- BehaviorModel interface (these would conflict with DynamicModel!)
	virtual void init();

	virtual void reinit();

    virtual void handleEvent(sofa::core::objectmodel::Event* event);

	/// Computation of a new simulation step.
	virtual void updatePosition(double dt);

	sofa::defaulttype::Vector3 currentForce;
};

#endif
