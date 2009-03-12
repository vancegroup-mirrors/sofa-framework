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
#ifndef SOFA_TYPEDEF_CONSTRAINT_FLOAT_H
#define SOFA_TYPEDEF_CONSTRAINT_FLOAT_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>


//Typedef to easily use constraints with float type 
#include <sofa/component/constraint/AttachConstraint.h>
#include <sofa/component/constraint/FixedConstraint.h>
#include <sofa/component/constraint/FixedPlaneConstraint.h>
#include <sofa/component/constraint/LinearMovementConstraint.h>
#include <sofa/component/constraint/OscillatorConstraint.h>
#include <sofa/component/constraint/ParabolicConstraint.h>
	   
//Attach Contraint
//---------------------
//Deformable
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec1fTypes> AttachConstraint1f;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec2fTypes> AttachConstraint2f;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec3fTypes> AttachConstraint3f;
//---------------------
//Rigid
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Rigid2fTypes> AttachConstraintRigid2f;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Rigid3fTypes> AttachConstraintRigid3f;

//Fixed Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec1fTypes> FixedConstraint1f;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec2fTypes> FixedConstraint2f;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec3fTypes> FixedConstraint3f;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec6fTypes> FixedConstraint6f;
//---------------------
//Rigid
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Rigid2fTypes> FixedConstraintRigid2f;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Rigid3fTypes> FixedConstraintRigid3f;


//FixedPlane Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::FixedPlaneConstraint<sofa::defaulttype::Vec3fTypes> FixedPlaneConstraint3f;

//Linear Movement Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec1fTypes> LinearMovementConstraint1f;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec2fTypes> LinearMovementConstraint2f;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec3fTypes> LinearMovementConstraint3f;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec6fTypes> LinearMovementConstraint6f;
//---------------------
//Rigid
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Rigid3fTypes> LinearMovementConstraintRigid3f;

//Parabolic Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::ParabolicConstraint<sofa::defaulttype::Vec3fTypes> ParabolicConstraint3f;
//---------------------
//Rigid
typedef sofa::component::constraint::ParabolicConstraint<sofa::defaulttype::Rigid3fTypes> ParabolicConstraintRigid3f;


//Oscillator Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::OscillatorConstraint<sofa::defaulttype::Vec3fTypes> OscillatorConstraint3f;
//---------------------
//Rigid
typedef sofa::component::constraint::OscillatorConstraint<sofa::defaulttype::Rigid3fTypes> OscillatorConstraintRigid3f;

#ifdef SOFA_FLOAT
typedef AttachConstraint1f			    AttachConstraint1;                               
typedef AttachConstraint2f			    AttachConstraint2;				 
typedef AttachConstraint3f			    AttachConstraint3;				 
typedef AttachConstraintRigid2f		            AttachConstraintRigid2;				 
typedef AttachConstraintRigid3f		            AttachConstraintRigid3;				 
typedef FixedConstraint1f			    FixedConstraint1;				 
typedef FixedConstraint2f			    FixedConstraint2;				 
typedef FixedConstraint3f			    FixedConstraint3;				 
typedef FixedConstraint6f			    FixedConstraint6;				 
typedef FixedConstraintRigid2f			    FixedConstraintRigid2;				 
typedef FixedConstraintRigid3f			    FixedConstraintRigid3;				 
typedef FixedPlaneConstraint3f			    FixedPlaneConstraint3;				 
typedef LinearMovementConstraint1f		    LinearMovementConstraint1;			 
typedef LinearMovementConstraint2f		    LinearMovementConstraint2;			 
typedef LinearMovementConstraint3f		    LinearMovementConstraint3;			 
typedef LinearMovementConstraint6f		    LinearMovementConstraint6;			 
typedef LinearMovementConstraintRigid3f	            LinearMovementConstraintRigid3;			 
typedef OscillatorConstraint3f			    OscillatorConstraint3;				 
typedef OscillatorConstraintRigid3f		    OscillatorConstraintRigid3;			 
#endif

#endif
