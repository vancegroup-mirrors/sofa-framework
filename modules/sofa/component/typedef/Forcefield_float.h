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
#ifndef SOFA_TYPEDEF_FORCEFIELD_FLOAT_H
#define SOFA_TYPEDEF_FORCEFIELD_FLOAT_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>

#include <sofa/component/forcefield/BeamFEMForceField.h>
#include <sofa/component/forcefield/BoxStiffSpringForceField.h>
#include <sofa/component/forcefield/ConstantForceField.h>
#include <sofa/component/forcefield/EdgePressureForceField.h>
#include <sofa/component/forcefield/LennardJonesForceField.h>
#include <sofa/component/forcefield/SPHFluidForceField.h>
#include <sofa/component/forcefield/PlaneForceField.h>
#include <sofa/component/forcefield/SphereForceField.h>
#include <sofa/component/forcefield/ConicalForceField.h>
#include <sofa/component/forcefield/EllipsoidForceField.h>
#include <sofa/component/forcefield/QuadBendingSprings.h>
#include <sofa/component/forcefield/SpringForceField.h>
#include <sofa/component/forcefield/StiffSpringForceField.h>
#include <sofa/component/forcefield/JointSpringForceField.h>
#include <sofa/component/forcefield/MeshSpringForceField.h>
#include <sofa/component/forcefield/RegularGridSpringForceField.h>
#include <sofa/component/forcefield/TensorForceField.h>
#include <sofa/component/forcefield/TetrahedronFEMForceField.h>
#include <sofa/component/forcefield/HexahedronFEMForceField.h>
#include <sofa/component/forcefield/TriangleBendingSprings.h> 
#include <sofa/component/forcefield/TriangleFEMForceField.h> 
#include <sofa/component/forcefield/TrianglePressureForceField.h> 
#include <sofa/component/forcefield/VectorSpringForceField.h>


//BeamFEMForceField
//---------------------
//Rigid
typedef sofa::component::forcefield::BeamFEMForceField<sofa::defaulttype::Rigid3fTypes> BeamFEMForceFieldRigid3f;

//BoxStiffSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::BoxStiffSpringForceField<sofa::defaulttype::Vec3fTypes> BoxStiffSpringForceField3f;

//ConicalForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::ConicalForceField<sofa::defaulttype::Vec3fTypes> ConicalForceField3f;

//ConstantForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec1fTypes> ConstantForceField1f;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec2fTypes> ConstantForceField2f;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec3fTypes> ConstantForceField3f;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec6fTypes> ConstantForceField6f;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Rigid2fTypes> ConstantForceFieldRigid2f;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Rigid3fTypes> ConstantForceFieldRigid3f;

//EdgePressureForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::EdgePressureForceField<sofa::defaulttype::Vec3fTypes> EdgePressureForceField3f;

//EllipsoidForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec1fTypes> EllipsoidForceField1f;
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec2fTypes> EllipsoidForceField2f;
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec3fTypes> EllipsoidForceField3f;


//JointSpringForceField
//---------------------
//Rigid
typedef sofa::component::forcefield::JointSpringForceField<sofa::defaulttype::Rigid3fTypes> JointSpringForceFieldRigid3f;

//LennardJonesForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::LennardJonesForceField<sofa::defaulttype::Vec3fTypes> LennardJonesForceField3f;


//MeshSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec1fTypes> MeshSpringForceField1f;
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec2fTypes> MeshSpringForceField2f;
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec3fTypes> MeshSpringForceField3f;

//PlaneForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec1fTypes> PlaneForceField1f;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec2fTypes> PlaneForceField2f;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec3fTypes> PlaneForceField3f;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec6fTypes> PlaneForceField6f;

//QuadBendingSprings
//---------------------
//Deformable
typedef sofa::component::forcefield::QuadBendingSprings<sofa::defaulttype::Vec2fTypes> QuadBendingSprings2f;
typedef sofa::component::forcefield::QuadBendingSprings<sofa::defaulttype::Vec3fTypes> QuadBendingSprings3f;


//RegularGridSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec1fTypes> RegularGridSpringForceField1f;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec2fTypes> RegularGridSpringForceField2f;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec3fTypes> RegularGridSpringForceField3f;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec6fTypes> RegularGridSpringForceField6f;


//SPHFluidForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SPHFluidForceField<sofa::defaulttype::Vec2fTypes> SPHFluidForceField2f;
typedef sofa::component::forcefield::SPHFluidForceField<sofa::defaulttype::Vec3fTypes> SPHFluidForceField3f;


//SphereForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec1fTypes> SphereForceField1f;
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec2fTypes> SphereForceField2f;
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec3fTypes> SphereForceField3f;

//SpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec1fTypes> SpringForceField1f;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec2fTypes> SpringForceField2f;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec3fTypes> SpringForceField3f;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec6fTypes> SpringForceField6f;


//StiffSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec1fTypes> StiffSpringForceField1f;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec2fTypes> StiffSpringForceField2f;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec3fTypes> StiffSpringForceField3f;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec6fTypes> StiffSpringForceField6f;


//TensorForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TensorForceField<sofa::defaulttype::Vec3fTypes> TensorForceField3f;


//TetrahedronFEMForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TetrahedronFEMForceField<sofa::defaulttype::Vec3fTypes> TetrahedronFEMForceField3f;


//TriangleBendingSprings
//---------------------
//Deformable
typedef sofa::component::forcefield::TriangleBendingSprings<sofa::defaulttype::Vec2fTypes> TriangleBendingSprings2f;
typedef sofa::component::forcefield::TriangleBendingSprings<sofa::defaulttype::Vec3fTypes> TriangleBendingSprings3f;

//TriangleFEMForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TriangleFEMForceField<sofa::defaulttype::Vec3fTypes> TriangleFEMForceField3f;

//VectorSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::VectorSpringForceField<sofa::defaulttype::Vec3fTypes> VectorSpringForceField3f;


#ifdef SOFA_FLOAT
typedef BeamFEMForceFieldRigid3f                          BeamFEMForceFieldRigid3;                         
typedef BoxStiffSpringForceField3f 			  BoxStiffSpringForceField3;
typedef ConicalForceField3f 				  ConicalForceField3;
typedef ConstantForceField1f 				  ConstantForceField1;
typedef ConstantForceField2f 				  ConstantForceField2;
typedef ConstantForceField3f 				  ConstantForceField3;
typedef ConstantForceField6f 				  ConstantForceField6;
typedef ConstantForceFieldRigid2f 			  ConstantForceFieldRigid2;
typedef ConstantForceFieldRigid3f 			  ConstantForceFieldRigid3;
typedef EdgePressureForceField3f 			  EdgePressureForceField3;
typedef EllipsoidForceField1f 				  EllipsoidForceField1;
typedef EllipsoidForceField2f 				  EllipsoidForceField2;
typedef EllipsoidForceField3f 				  EllipsoidForceField3;
typedef JointSpringForceFieldRigid3f 			  JointSpringForceFieldRigid3;
typedef LennardJonesForceField3f 			  LennardJonesForceField3;
typedef MeshSpringForceField1f 				  MeshSpringForceField1;
typedef MeshSpringForceField2f 				  MeshSpringForceField2;
typedef MeshSpringForceField3f 				  MeshSpringForceField3;
typedef PlaneForceField1f  				  PlaneForceField1;
typedef PlaneForceField2f 				  PlaneForceField2;
typedef PlaneForceField3f 				  PlaneForceField3;
typedef PlaneForceField6f 				  PlaneForceField6;
typedef QuadBendingSprings2f 				  QuadBendingSprings2;
typedef QuadBendingSprings3f 				  QuadBendingSprings3;
typedef RegularGridSpringForceField1f 			  RegularGridSpringForceField1;
typedef RegularGridSpringForceField2f 			  RegularGridSpringForceField2;
typedef RegularGridSpringForceField3f 			  RegularGridSpringForceField3;
typedef RegularGridSpringForceField6f 			  RegularGridSpringForceField6;
typedef SPHFluidForceField2f 				  SPHFluidForceField2;
typedef SPHFluidForceField3f 				  SPHFluidForceField3;
typedef SphereForceField1f 				  SphereForceField1;
typedef SphereForceField2f 				  SphereForceField2;
typedef SphereForceField3f 				  SphereForceField3;
typedef SpringForceField1f 				  SpringForceField1;
typedef SpringForceField2f 				  SpringForceField2;
typedef SpringForceField3f 				  SpringForceField3;
typedef SpringForceField6f 				  SpringForceField6;
typedef StiffSpringForceField1f 			  StiffSpringForceField1;
typedef StiffSpringForceField2f 			  StiffSpringForceField2;
typedef StiffSpringForceField3f 			  StiffSpringForceField3;
typedef StiffSpringForceField6f 			  StiffSpringForceField6;
typedef TensorForceField3f 				  TensorForceField3;
typedef TetrahedronFEMForceField3f 			  TetrahedronFEMForceField3;
typedef TriangleBendingSprings2f			  TriangleBendingSprings2;
typedef TriangleBendingSprings3f			  TriangleBendingSprings3;
typedef TriangleFEMForceField3f			          TriangleFEMForceField3;
typedef VectorSpringForceField3f			  VectorSpringForceField3;
#endif

#endif
