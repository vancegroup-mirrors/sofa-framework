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
#ifndef SOFA_TYPEDEF_FORCEFIELD_DOUBLE_H
#define SOFA_TYPEDEF_FORCEFIELD_DOUBLE_H

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
typedef sofa::component::forcefield::BeamFEMForceField<sofa::defaulttype::Rigid3dTypes> BeamFEMForceFieldRigid3d;

//BoxStiffSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::BoxStiffSpringForceField<sofa::defaulttype::Vec3dTypes> BoxStiffSpringForceField3d;

//ConicalForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::ConicalForceField<sofa::defaulttype::Vec3dTypes> ConicalForceField3d;

//ConstantForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec1dTypes> ConstantForceField1d;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec2dTypes> ConstantForceField2d;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec3dTypes> ConstantForceField3d;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Vec6dTypes> ConstantForceField6d;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Rigid2dTypes> ConstantForceFieldRigid2d;
typedef sofa::component::forcefield::ConstantForceField<sofa::defaulttype::Rigid3dTypes> ConstantForceFieldRigid3d;

//EdgePressureForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::EdgePressureForceField<sofa::defaulttype::Vec3dTypes> EdgePressureForceField3d;

//EllipsoidForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec1dTypes> EllipsoidForceField1d;
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec2dTypes> EllipsoidForceField2d;
typedef sofa::component::forcefield::EllipsoidForceField<sofa::defaulttype::Vec3dTypes> EllipsoidForceField3d;


//JointSpringForceField
//---------------------
//Rigid
typedef sofa::component::forcefield::JointSpringForceField<sofa::defaulttype::Rigid3dTypes> JointSpringForceFieldRigid3d;

//LennardJonesForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::LennardJonesForceField<sofa::defaulttype::Vec3dTypes> LennardJonesForceField3d;


//MeshSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec1dTypes> MeshSpringForceField1d;
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec2dTypes> MeshSpringForceField2d;
typedef sofa::component::forcefield::MeshSpringForceField<sofa::defaulttype::Vec3dTypes> MeshSpringForceField3d;

//PlaneForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec1dTypes> PlaneForceField1d;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec2dTypes> PlaneForceField2d;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec3dTypes> PlaneForceField3d;
typedef sofa::component::forcefield::PlaneForceField<sofa::defaulttype::Vec6dTypes> PlaneForceField6d;

//QuadBendingSprings
//---------------------
//Deformable
typedef sofa::component::forcefield::QuadBendingSprings<sofa::defaulttype::Vec2dTypes> QuadBendingSprings2d;
typedef sofa::component::forcefield::QuadBendingSprings<sofa::defaulttype::Vec3dTypes> QuadBendingSprings3d;


//RegularGridSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec1dTypes> RegularGridSpringForceField1d;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec2dTypes> RegularGridSpringForceField2d;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec3dTypes> RegularGridSpringForceField3d;
typedef sofa::component::forcefield::RegularGridSpringForceField<sofa::defaulttype::Vec6dTypes> RegularGridSpringForceField6d;


//SPHFluidForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SPHFluidForceField<sofa::defaulttype::Vec2dTypes> SPHFluidForceField2d;
typedef sofa::component::forcefield::SPHFluidForceField<sofa::defaulttype::Vec3dTypes> SPHFluidForceField3d;


//SphereForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec1dTypes> SphereForceField1d;
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec2dTypes> SphereForceField2d;
typedef sofa::component::forcefield::SphereForceField<sofa::defaulttype::Vec3dTypes> SphereForceField3d;

//SpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec1dTypes> SpringForceField1d;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec2dTypes> SpringForceField2d;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec3dTypes> SpringForceField3d;
typedef sofa::component::forcefield::SpringForceField<sofa::defaulttype::Vec6dTypes> SpringForceField6d;


//StiffSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec1dTypes> StiffSpringForceField1d;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec2dTypes> StiffSpringForceField2d;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec3dTypes> StiffSpringForceField3d;
typedef sofa::component::forcefield::StiffSpringForceField<sofa::defaulttype::Vec6dTypes> StiffSpringForceField6d;


//TensorForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TensorForceField<sofa::defaulttype::Vec3dTypes> TensorForceField3d;


//TetrahedronFEMForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TetrahedronFEMForceField<sofa::defaulttype::Vec3dTypes> TetrahedronFEMForceField3d;


//TriangleBendingSprings
//---------------------
//Deformable
typedef sofa::component::forcefield::TriangleBendingSprings<sofa::defaulttype::Vec2dTypes> TriangleBendingSprings2d;
typedef sofa::component::forcefield::TriangleBendingSprings<sofa::defaulttype::Vec3dTypes> TriangleBendingSprings3d;

//TriangleFEMForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::TriangleFEMForceField<sofa::defaulttype::Vec3dTypes> TriangleFEMForceField3d;

//VectorSpringForceField
//---------------------
//Deformable
typedef sofa::component::forcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes> VectorSpringForceField3d;


#ifndef SOFA_FLOAT
typedef BeamFEMForceFieldRigid3d                          BeamFEMForceFieldRigid3;                         
typedef BoxStiffSpringForceField3d 			  BoxStiffSpringForceField3;
typedef ConicalForceField3d 				  ConicalForceField3;
typedef ConstantForceField1d 				  ConstantForceField1;
typedef ConstantForceField2d 				  ConstantForceField2;
typedef ConstantForceField3d 				  ConstantForceField3;
typedef ConstantForceField6d 				  ConstantForceField6;
typedef ConstantForceFieldRigid2d 			  ConstantForceFieldRigid2;
typedef ConstantForceFieldRigid3d 			  ConstantForceFieldRigid3;
typedef EdgePressureForceField3d 			  EdgePressureForceField3;
typedef EllipsoidForceField1d 				  EllipsoidForceField1;
typedef EllipsoidForceField2d 				  EllipsoidForceField2;
typedef EllipsoidForceField3d 				  EllipsoidForceField3;
typedef JointSpringForceFieldRigid3d 			  JointSpringForceFieldRigid3;
typedef LennardJonesForceField3d 			  LennardJonesForceField3;
typedef MeshSpringForceField1d 				  MeshSpringForceField1;
typedef MeshSpringForceField2d 				  MeshSpringForceField2;
typedef MeshSpringForceField3d 				  MeshSpringForceField3;
typedef PlaneForceField1d  				  PlaneForceField1;
typedef PlaneForceField2d 				  PlaneForceField2;
typedef PlaneForceField3d 				  PlaneForceField3;
typedef PlaneForceField6d 				  PlaneForceField6;
typedef QuadBendingSprings2d 				  QuadBendingSprings2;
typedef QuadBendingSprings3d 				  QuadBendingSprings3;
typedef RegularGridSpringForceField1d 			  RegularGridSpringForceField1;
typedef RegularGridSpringForceField2d 			  RegularGridSpringForceField2;
typedef RegularGridSpringForceField3d 			  RegularGridSpringForceField3;
typedef RegularGridSpringForceField6d 			  RegularGridSpringForceField6;
typedef SPHFluidForceField2d 				  SPHFluidForceField2;
typedef SPHFluidForceField3d 				  SPHFluidForceField3;
typedef SphereForceField1d 				  SphereForceField1;
typedef SphereForceField2d 				  SphereForceField2;
typedef SphereForceField3d 				  SphereForceField3;
typedef SpringForceField1d 				  SpringForceField1;
typedef SpringForceField2d 				  SpringForceField2;
typedef SpringForceField3d 				  SpringForceField3;
typedef SpringForceField6d 				  SpringForceField6;
typedef StiffSpringForceField1d 			  StiffSpringForceField1;
typedef StiffSpringForceField2d 			  StiffSpringForceField2;
typedef StiffSpringForceField3d 			  StiffSpringForceField3;
typedef StiffSpringForceField6d 			  StiffSpringForceField6;
typedef TensorForceField3d 				  TensorForceField3;
typedef TetrahedronFEMForceField3d 			  TetrahedronFEMForceField3;
typedef TriangleBendingSprings2d			  TriangleBendingSprings2;
typedef TriangleBendingSprings3d			  TriangleBendingSprings3;
typedef TriangleFEMForceField3d			          TriangleFEMForceField3;
typedef VectorSpringForceField3d			  VectorSpringForceField3;
#endif

#endif
