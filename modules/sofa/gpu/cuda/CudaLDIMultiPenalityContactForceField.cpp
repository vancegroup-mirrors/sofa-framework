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

#include "CudaLDIMultiPenalityContactForceField.h"

#include "CudaTypes.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/collision/CubeModel.h>
#include <fstream>
#include <sofa/helper/gl/template.h>


namespace sofa {

namespace gpu {

namespace cuda {

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
void CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2,ResponseDataTypes>::init()
{
	this->getContext()->get(rasterizer);
	if (rasterizer == NULL)
	{
		serr << "ERROR: CudaRasterizer not found" << sendl;
		return;
	}

	
}

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
void CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2, ResponseDataTypes>::addForce()
{
  const helper::vector<MState*> vmstate = rasterizer->getMState();
  CudaVector<int> pairFlags = rasterizer->getPairFlags();
  CudaVector<short> pairIndex = rasterizer->getPairIndex();
  unsigned nbObj = rasterizer->getNbObj();
  unsigned nbPair = rasterizer->getNbPairs();

  if (nbPair>0) {
    this->lesForceFields.clear();

    for (unsigned obj0 = 0; obj0 < nbObj; ++obj0)
	    for (unsigned obj1 = 0; obj1 < nbObj; ++obj1)
	    {
		    int i = obj0*nbObj+obj1;
		    if (pairFlags[i>>5]&(1<<(i&31)))
		    {
			    //printf("CudaLDIMultiPenalityContactForceField::create ForceField %d %d\n",obj0,obj1);
			    int po = (int) pairIndex[i];
			    InteractionForceField * ff = new CudaLDIPenalityContactForceField<ResponseDataTypes,ResponseDataTypes,ResponseDataTypes>(vmstate[obj0],vmstate[obj1],rasterizer,po);
			    this->lesForceFields.push_back(ff);
		    }
	    }
  }

  for (unsigned i=0;i<this->lesForceFields.size();i++) lesForceFields[i]->addForce();
}

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
void CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2, ResponseDataTypes>::addDForce(double d1,double d2)
{
  for (unsigned i=0;i<this->lesForceFields.size();i++) lesForceFields[i]->addDForce(d1,d2);
}

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
void CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2, ResponseDataTypes>::addDForceV(double d1,double d2)
{
  for (unsigned i=0;i<this->lesForceFields.size();i++) lesForceFields[i]->addDForceV(d1,d2);
}

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
BaseMechanicalState* CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2, ResponseDataTypes>::getMechModel1()
{
    return NULL;
}

template<class DataTypes1,class DataTypes2, class ResponseDataTypes>
BaseMechanicalState* CudaLDIMultiPenalityContactForceField<DataTypes1,DataTypes2, ResponseDataTypes>::getMechModel2()
{
    return NULL;
}

using namespace sofa::defaulttype;
using namespace sofa::core;

int CudaLDIMultiPenalityContactForceFieldClass = RegisterObject("CUDA LDI Contact")
#ifndef SOFA_FLOAT
.add< CudaLDIMultiPenalityContactForceField<Vec3dTypes,Vec3dTypes,Vec3dTypes> >()
//.add< CudaLDIPenalityContactForceField<Rigid3dTypes,Vec3dTypes,CudaVec3fTypes> >()
//.add< CudaLDIPenalityContactForceField<Vec3dTypes,Rigid3dTypes,CudaVec3fTypes> >()
//.add< CudaLDIPenalityContactForceField<Rigid3dTypes,Rigid3dTypes,CudaVec3fTypes> >()
#endif
#ifndef SOFA_DOUBLE
.add< CudaLDIMultiPenalityContactForceField<Vec3fTypes,Vec3fTypes,Vec3fTypes> >(true)
//.add< CudaLDIPenalityContactForceField<Rigid3fTypes,Vec3fTypes,CudaVec3fTypes> >()
//.add< CudaLDIPenalityContactForceField<Vec3fTypes,Rigid3fTypes,CudaVec3fTypes> >()
//.add< CudaLDIPenalityContactForceField<Rigid3fTypes,Rigid3fTypes,CudaVec3fTypes> >()
#endif
.add< CudaLDIMultiPenalityContactForceField<CudaVec3fTypes,CudaVec3fTypes,CudaVec3fTypes> >()
#ifdef SOFA_GPU_CUDA_DOUBLE
.add< CudaLDIMultiPenalityContactForceField<CudaVec3dTypes,CudaVec3dTypes,CudaVec3dTypes> >()
#endif
;
/*
#ifndef SOFA_FLOAT
template class CudaLDIMultiPenalityContactForceField<Vec3dTypes,Vec3dTypes,Vec3dTypes>;
//template class CudaLDIPenalityContactForceField<Rigid3dTypes,Vec3dTypes,CudaVec3fTypes>;
//template class CudaLDIPenalityContactForceField<Vec3dTypes,Rigid3dTypes,CudaVec3fTypes>;
//template class CudaLDIPenalityContactForceField<Rigid3dTypes,Rigid3dTypes,CudaVec3fTypes>;
#endif
#ifndef SOFA_DOUBLE
template class CudaLDIMultiPenalityContactForceField<Vec3fTypes,Vec3fTypes,Vec3fTypes>;
// template class CudaLDIPenalityContactForceField<Rigid3fTypes,Vec3fTypes,CudaVec3fTypes>;
// template class CudaLDIPenalityContactForceField<Vec3fTypes,Rigid3fTypes,CudaVec3fTypes>;
// template class CudaLDIPenalityContactForceField<Rigid3fTypes,Rigid3fTypes,CudaVec3fTypes>;
#endif
template class CudaLDIMultiPenalityContactForceField<CudaVec3fTypes,CudaVec3fTypes,CudaVec3fTypes>;
#ifdef SOFA_GPU_CUDA_DOUBLE
template class CudaLDIMultiPenalityContactForceField<CudaVec3dTypes,CudaVec3dTypes,CudaVec3dTypes>;
#endif
*/
} // namespace cuda

} // namespace gpu

} // namespace sofa
