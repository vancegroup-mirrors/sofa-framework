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
#include "CudaTestForceField.h"
#include <sofa/core/componentmodel/behavior/ForceField.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace gpu
{

namespace cuda
{

SOFA_DECL_CLASS(CudaTestForceField)

int CudaTestForceFieldCudaClass = core::RegisterObject("GPU-side test forcefield using CUDA")
.add< CudaTestForceField >()
;

extern "C"
{
void CudaTestForceField3f_addForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* f, const void* x, const void* v);
void CudaTestForceField3f_addDForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* df, const void* dx);
}

CudaTestForceField::CudaTestForceField()
: nbVertex(0), nbElementPerVertex(0)
{
}

void CudaTestForceField::init()
{
    core::componentmodel::behavior::ForceField<CudaVec3fTypes>::init();
    reinit();
}

void CudaTestForceField::reinit()
{
    sofa::core::componentmodel::topology::BaseMeshTopology* topology = getContext()->get<sofa::core::componentmodel::topology::BaseMeshTopology>();
#ifdef SOFA_NEW_HEXA 
    if (topology==NULL || topology->getNbHexas()==0)
#else
    if (topology==NULL || topology->getNbCubes()==0)
#endif    
    {
      serr << "ERROR(CudaTestForceField): no elements found.\n";
	return;
    }
#ifdef SOFA_NEW_HEXA    
    const VecElement& inputElems = topology->getHexas();
#else
    const VecElement& inputElems = topology->getCubes();
#endif
    std::map<int,int> nelems; // number of elements attached to each node
    for (unsigned int i=0;i<inputElems.size();i++)
    {
        const Element& e = inputElems[i];
        for (unsigned int j=0;j<e.size();j++)
            ++nelems[e[j]];
    }
    int nmax = 0;
    for (std::map<int,int>::const_iterator it = nelems.begin(); it != nelems.end(); ++it)
        if (it->second > nmax)
            nmax = it->second;
    int nbv = 0;
    if (!nelems.empty())
    {
        nbv = nelems.rbegin()->first + 1;
    }
    sout << "CudaTestForceField: "<<inputElems.size()<<" elements, "<<nbv<<" nodes, max "<<nmax<<" elements per node."<<sendl;
    init(inputElems.size(), nbv, nmax);
    sout << "CudaTestForceField: precomputations..."<<sendl;
    const VecCoord&x = *this->mstate->getX();
    nelems.clear();
    for (unsigned int i=0;i<inputElems.size();i++)
    {
        const Element& e = inputElems[i];
	//computeMaterialStiffness(i,e);
	Coord center;
	center = x[e[0]];
	for (unsigned int j=1;j<e.size();++j)
	    center += x[e[1]];
	center /= e.size();
	float r = 0;
	for (unsigned int j=0;j<e.size();++j)
	    r += (x[e[j]]-center).norm();
	r /= e.size();
	sout << "E["<<i<<"]: center="<<center<<" r="<<r<<sendl;
        setE(i, e, 1, r, 10, 10);
        for (unsigned int j=0;j<e.size();j++)
            setV(e[j], nelems[e[j]]++, i*e.size()+j);
    }
    sout << "CudaTestForceField::reinit() DONE."<<sendl;
}

void CudaTestForceField::addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v)
{
    f.resize(x.size());
    CudaTestForceField3f_addForce(
        elems.size(),
        nbVertex,
        nbElementPerVertex,
        elems.deviceRead(),
        state.deviceWrite(),
        velems.deviceRead(),
        f.deviceWrite(),
        x.deviceRead(),
        v.deviceRead());
}

void CudaTestForceField::addDForce (VecDeriv& df, const VecDeriv& dx)
{
    df.resize(dx.size());
    CudaTestForceField3f_addDForce(
        elems.size(),
        nbVertex,
        nbElementPerVertex,
        elems.deviceRead(),
        state.deviceWrite(),
        velems.deviceRead(),
        df.deviceWrite(),
        dx.deviceRead());
}

} // namespace cuda

} // namespace gpu

} // namespace sofa
