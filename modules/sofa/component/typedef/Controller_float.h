/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2010 MGH, INRIA, USTL, UJF, CNRS                    *
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



// File automatically generated by "generateTypedef"


#ifndef SOFA_TYPEDEF_Controller_float_H
#define SOFA_TYPEDEF_Controller_float_H

//Default files containing the declaration of the vector type
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>


#ifdef SOFA_GPU_CUDA
#include <sofa/gpu/cuda/CudaTypesBase.h>
#include <sofa/gpu/cuda/CudaTypes.h>
#endif
#ifdef SOFA_GPU_OPENCL
#include <sofa/gpu/opencl/OpenCLTypes.h>
#endif


#include <sofa/component/collision/ComplianceMatrixUpdateManager.h>
#include <sofa/component/collision/ComplianceMatrixUpdateManagerCarving.h>
#include <sofa/component/controller/EdgeSetController.h>
#include <sofa/component/controller/HandStateController.h>
#include <sofa/component/controller/JointSpringController.h>
#include <sofa/component/controller/LCPForceFeedback.h>
#include <sofa/component/controller/MechanicalStateController.h>



//---------------------------------------------------------------------------------------------
//Typedef for ComplianceMatrixUpdateManager
typedef sofa::component::collision::ComplianceMatrixUpdateManager<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ComplianceMatrixUpdateManager3f;



//---------------------------------------------------------------------------------------------
//Typedef for ComplianceMatrixUpdateManagerCarving
typedef sofa::component::collision::ComplianceMatrixUpdateManagerCarving<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ComplianceMatrixUpdateManagerCarving3f;



//---------------------------------------------------------------------------------------------
//Typedef for EdgeSetController
typedef sofa::component::controller::EdgeSetController<sofa::defaulttype::StdRigidTypes<3, float> > EdgeSetControllerRigid3f;



//---------------------------------------------------------------------------------------------
//Typedef for HandStateController
typedef sofa::component::controller::HandStateController<sofa::defaulttype::StdRigidTypes<3, float> > HandStateControllerRigid3f;
typedef sofa::component::controller::HandStateController<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > HandStateController1f;



//---------------------------------------------------------------------------------------------
//Typedef for JointSpringController
typedef sofa::component::controller::JointSpringController<sofa::defaulttype::StdRigidTypes<3, float> > JointSpringControllerRigid3f;



//---------------------------------------------------------------------------------------------
//Typedef for LCPForceFeedback
typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::StdRigidTypes<3, float> > LCPForceFeedbackRigid3f;
typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > LCPForceFeedback1f;



//---------------------------------------------------------------------------------------------
//Typedef for MechanicalStateController
typedef sofa::component::controller::MechanicalStateController<sofa::defaulttype::StdRigidTypes<3, float> > MechanicalStateControllerRigid3f;
typedef sofa::component::controller::MechanicalStateController<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > MechanicalStateController1f;





#ifdef SOFA_FLOAT
typedef ComplianceMatrixUpdateManager3f ComplianceMatrixUpdateManager3;
typedef ComplianceMatrixUpdateManagerCarving3f ComplianceMatrixUpdateManagerCarving3;
typedef EdgeSetControllerRigid3f EdgeSetControllerRigid3;
typedef HandStateControllerRigid3f HandStateControllerRigid3;
typedef HandStateController1f HandStateController1;
typedef JointSpringControllerRigid3f JointSpringControllerRigid3;
typedef LCPForceFeedbackRigid3f LCPForceFeedbackRigid3;
typedef LCPForceFeedback1f LCPForceFeedback1;
typedef MechanicalStateControllerRigid3f MechanicalStateControllerRigid3;
typedef MechanicalStateController1f MechanicalStateController1;
#endif

#endif
