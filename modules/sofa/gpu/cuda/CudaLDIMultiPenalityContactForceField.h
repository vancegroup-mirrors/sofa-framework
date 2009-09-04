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
#ifndef SOFA_GPU_CUDA_CudaLDIMultiPenalityContactForceField_H
#define SOFA_GPU_CUDA_CudaLDIMultiPenalityContactForceField_H

#include "CudaTypes.h"
#include "CudaRasterizer.h"

#include <sofa/core/CollisionModel.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/core/objectmodel/DataFileName.h>


#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/core/componentmodel/behavior/InteractionForceField.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/componentmodel/behavior/MixedInteractionForceField.h>

namespace sofa {

namespace gpu {

namespace cuda {

using namespace sofa::core::componentmodel::behavior;
using namespace sofa::core::objectmodel;

template<class DataTypes1, class DataTypes2, class ResponseDataTypes>
class CudaLDIMultiPenalityContactForceField : public InteractionForceField , public virtual BaseObject
{
public:
	typedef typename DataTypes1::VecCoord VecCoord1;
	typedef typename DataTypes1::VecDeriv VecDeriv1;
	typedef typename DataTypes1::Coord Coord1;
	typedef typename DataTypes1::Deriv Deriv1;
	typedef MechanicalState<DataTypes1> MechanicalState1;
	typedef typename DataTypes2::VecCoord VecCoord2;
	typedef typename DataTypes2::VecDeriv VecDeriv2;
	typedef typename DataTypes2::Coord Coord2;
	typedef typename DataTypes2::Deriv Deriv2;
	typedef MechanicalState<DataTypes2> MechanicalState2;
	typedef float Real;
	typedef MechanicalState<ResponseDataTypes> MState;

protected:
	CudaRasterizer<ResponseDataTypes> * rasterizer;
	std::vector<InteractionForceField *> lesForceFields;
public:
	Data<Real> stiffness;

      CudaLDIMultiPenalityContactForceField()
	: rasterizer(NULL)
	  {}

	virtual void init();
	
	virtual void addForce();
        virtual void addDForce(double d1,double d2);
	virtual void addDForceV(double d1,double d2);
	
	virtual double getPotentialEnergy()
	{	  
	  std::cerr<<"CudaLDIPenalityContactForceField::getPotentialEnergy-not-implemented !!!"<<std::endl;
	  return 0;
	}
	
 
	void draw() {}

	BaseMechanicalState* getMechModel1();
	BaseMechanicalState* getMechModel2();
};

} // namespace cuda

} // namespace gpu

} // namespace sofa

#endif
