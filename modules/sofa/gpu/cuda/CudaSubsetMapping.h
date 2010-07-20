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
#ifndef SOFA_GPU_CUDA_CUDASUBSETMAPPING_H
#define SOFA_GPU_CUDA_CUDASUBSETMAPPING_H

#include "CudaTypes.h"
#include <sofa/component/mapping/SubsetMapping.h>
#include <sofa/core/behavior/MappedModel.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/MechanicalMapping.h>

namespace sofa
{

namespace component
{

namespace mapping
{

template <>
class SubsetMappingInternalData<gpu::cuda::CudaVec3fTypes, gpu::cuda::CudaVec3fTypes>
{
public:
    sofa::gpu::cuda::CudaVector<int> map;
    int maxNOut;
    sofa::gpu::cuda::CudaVector<int> mapT;
    SubsetMappingInternalData() : maxNOut(0)
    {}

    void clear(int reserve=0)
    {
        map.clear();
        map.reserve(reserve);
    }

    int addPoint(int fromIndex)
    {
        int i = map.size();
        map.resize(i+1);
        map[i] = fromIndex;
        return i;
    }

    void init(int insize)
    {
	unsigned int n = map.size();
	std::vector<int> nout;
        if (n==0) return;
	if (n==1)
	    maxNOut = 1;
	else
	{
	    // compute mapT
	    nout.resize(insize);
	    for (unsigned int i=0;i<map.size();i++)
		nout[map[i]]++;
	    for (int i=0;i<insize;i++)
		if (nout[i] > maxNOut) maxNOut = nout[i];
	}
        if (maxNOut <= 1)
        {
            std::cout << "CudaSubsetMapping: strict subset, no need for mapT."<<std::endl;
            // at most one duplicated points per input. mapT is not necessary
            mapT.clear();
        }
        else
        {
            int nbloc = (insize+BSIZE-1)/BSIZE;
            std::cout << "CudaSubsetMapping: mapT with "<<maxNOut<<" entries per DOF and "<<nbloc<<" blocs."<<std::endl;
            mapT.resize(nbloc*(BSIZE*maxNOut));
            for (unsigned int i=0;i<mapT.size();i++)
                mapT[i] = -1;
            nout.clear();
            nout.resize(insize);
            for (unsigned int i=0;i<map.size();i++)
            {
                int index = map[i];
                int num = nout[index]++;
                int b = (index / BSIZE); index -= b*BSIZE;
                mapT[(maxNOut*b+num)*BSIZE+index] = i;
            }
        }
    }
};

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::postInit();

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::postInit();

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );


//////// CudaVec3f1

template <>
class SubsetMappingInternalData<gpu::cuda::CudaVec3f1Types, gpu::cuda::CudaVec3f1Types> : public SubsetMappingInternalData<gpu::cuda::CudaVec3fTypes, gpu::cuda::CudaVec3fTypes>
{
};

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::postInit();

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::postInit();

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
class SubsetMappingInternalData<gpu::cuda::CudaVec3f1Types, gpu::cuda::CudaVec3fTypes> : public SubsetMappingInternalData<gpu::cuda::CudaVec3fTypes, gpu::cuda::CudaVec3fTypes>
{
};

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::postInit();

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::postInit();

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3f1Types>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
class SubsetMappingInternalData<gpu::cuda::CudaVec3fTypes, gpu::cuda::CudaVec3f1Types> : public SubsetMappingInternalData<gpu::cuda::CudaVec3fTypes, gpu::cuda::CudaVec3fTypes>
{
};

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::postInit();

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3f1Types> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::postInit();

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::apply( Out::VecCoord& out, const In::VecCoord& in );

template <>
void SubsetMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaVec3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3f1Types> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
