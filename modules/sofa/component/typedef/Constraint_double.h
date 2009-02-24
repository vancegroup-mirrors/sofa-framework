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
#ifndef SOFA_TYPEDEF_CONSTRAINT_DOUBLE_H
#define SOFA_TYPEDEF_CONSTRAINT_DOUBLE_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>



//Typedef to easily use constraints with double type 
#include <sofa/component/constraint/AttachConstraint.h>
#include <sofa/component/constraint/BoxConstraint.h>
#include <sofa/component/constraint/FixedConstraint.h>
#include <sofa/component/constraint/FixedPlaneConstraint.h>
#include <sofa/component/constraint/LinearMovementConstraint.h>
#include <sofa/component/constraint/LinearSolverConstraintCorrection.h>
#include <sofa/component/constraint/OscillatorConstraint.h>
#include <sofa/component/constraint/ParabolicConstraint.h>
	   
//Attach Contraint
//---------------------
//Deformable
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec1dTypes> AttachConstraint1d;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec2dTypes> AttachConstraint2d;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Vec3dTypes> AttachConstraint3d;
//---------------------
//Rigid
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Rigid2dTypes> AttachConstraintRigid2d;
typedef sofa::component::constraint::AttachConstraint<sofa::defaulttype::Rigid3dTypes> AttachConstraintRigid3d;

//Box Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::BoxConstraint<sofa::defaulttype::Vec3dTypes> BoxConstraint3d;

//Fixed Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec1dTypes> FixedConstraint1d;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec2dTypes> FixedConstraint2d;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec3dTypes> FixedConstraint3d;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Vec6dTypes> FixedConstraint6d;
//---------------------
//Rigid
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Rigid2dTypes> FixedConstraintRigid2d;
typedef sofa::component::constraint::FixedConstraint<sofa::defaulttype::Rigid3dTypes> FixedConstraintRigid3d;


//FixedPlane Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::FixedPlaneConstraint<sofa::defaulttype::Vec3dTypes> FixedPlaneConstraint3d;

//Linear Movement Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec1dTypes> LinearMovementConstraint1d;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec2dTypes> LinearMovementConstraint2d;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec3dTypes> LinearMovementConstraint3d;
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Vec6dTypes> LinearMovementConstraint6d;
//---------------------
//Rigid
typedef sofa::component::constraint::LinearMovementConstraint<sofa::defaulttype::Rigid3dTypes> LinearMovementConstraintRigid3d;

//Parabolic Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::ParabolicConstraint<sofa::defaulttype::Vec3dTypes> ParabolicConstraint3d;
//---------------------
//Rigid
typedef sofa::component::constraint::ParabolicConstraint<sofa::defaulttype::Rigid3dTypes> ParabolicConstraintRigid3d;

//Oscillator Constraint
//---------------------
//Deformable
typedef sofa::component::constraint::OscillatorConstraint<sofa::defaulttype::Vec3dTypes> OscillatorConstraint3d;
//---------------------
//Rigid
typedef sofa::component::constraint::OscillatorConstraint<sofa::defaulttype::Rigid3dTypes> OscillatorConstraintRigid3d;


#ifndef SOFA_FLOAT
typedef AttachConstraint1d			    AttachConstraint1;                               
typedef AttachConstraint2d			    AttachConstraint2;				 
typedef AttachConstraint3d			    AttachConstraint3;				 
typedef AttachConstraintRigid2d		            AttachConstraintRigid2;				 
typedef AttachConstraintRigid3d		            AttachConstraintRigid3;				 
typedef BoxConstraint3d			            BoxConstraint3;					 
typedef FixedConstraint1d			    FixedConstraint1;				 
typedef FixedConstraint2d			    FixedConstraint2;				 
typedef FixedConstraint3d			    FixedConstraint3;				 
typedef FixedConstraint6d			    FixedConstraint6;				 
typedef FixedConstraintRigid2d			    FixedConstraintRigid2;				 
typedef FixedConstraintRigid3d			    FixedConstraintRigid3;				 
typedef FixedPlaneConstraint3d			    FixedPlaneConstraint3;				 
typedef LinearMovementConstraint1d		    LinearMovementConstraint1;			 
typedef LinearMovementConstraint2d		    LinearMovementConstraint2;			 
typedef LinearMovementConstraint3d		    LinearMovementConstraint3;			 
typedef LinearMovementConstraint6d		    LinearMovementConstraint6;			 
typedef LinearMovementConstraintRigid3d	            LinearMovementConstraintRigid3;			 
typedef OscillatorConstraint3d			    OscillatorConstraint3;				 
typedef OscillatorConstraintRigid3d		    OscillatorConstraintRigid3;			 
#endif


#endif
