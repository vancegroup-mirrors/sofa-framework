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
#ifndef SOFA_GPU_CUDA_CUDATESTFORCEFIELD_H
#define SOFA_GPU_CUDA_CUDATESTFORCEFIELD_H

#include "CudaTypes.h"
#include <sofa/core/componentmodel/behavior/ForceField.h>
#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>

namespace sofa
{

namespace gpu
{

namespace cuda
{

using namespace sofa::defaulttype;

class CudaTestForceField : public core::componentmodel::behavior::ForceField<CudaVec3fTypes>
{
public:
    typedef CudaVec3fTypes::Real Real;
    typedef CudaVec3fTypes::Coord Coord;
#ifdef SOFA_NEW_HEXA    
    typedef sofa::core::componentmodel::topology::BaseMeshTopology::Hexa Element;
    typedef sofa::core::componentmodel::topology::BaseMeshTopology::SeqHexas VecElement;
#else    
    typedef sofa::core::componentmodel::topology::BaseMeshTopology::Cube Element;
    typedef sofa::core::componentmodel::topology::BaseMeshTopology::SeqCubes VecElement;
#endif
    /** Static data associated with each element
    */
    struct GPUElement
    {
        /// @name index of the 8 connected vertices
	/// @{
	int v[8];
	/// @}
        /// @name material stiffness
	/// @{
        float Kvol, Kr;
	/// @}
        /// @name initial volume and radius
	/// @{
        float initvol, initr;
	/// @}
    };

    CudaVector<GPUElement> elems;

    /// Varying data associated with each element
    struct GPUElementState
    {
        /// center
	Coord center;
	/// volume diff*Kvol
	float dv;
	float r,Kr;
	float dummy1,dummy2;
    };

    CudaVector<GPUElementState> state;

    int nbVertex; ///< number of vertices to process to compute all elements
    int nbElementPerVertex; ///< max number of elements connected to a vertex
    /// Index of elements attached to each points (layout per bloc of NBLOC vertices, with first element of each vertex, then second element, etc)
    /// No that each integer is actually equat the the index of the element * 8 + the index of this vertex inside the element.
    CudaVector<int> velems;

    CudaTestForceField();
    void init();
    void reinit();
    void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& /*v*/);
    void addDForce (VecDeriv& df, const VecDeriv& dx);
    double getPotentialEnergy(const VecCoord&) { return 0.0; }

protected:

    void init(int nbe, int nbv, int nbelemperv)
    {
        elems.resize(nbe);
        state.resize(nbe);
        nbVertex = nbv;
        nbElementPerVertex = nbelemperv;
        int nbloc = (nbVertex+BSIZE-1)/BSIZE;
        velems.resize(nbloc*nbElementPerVertex*BSIZE);
        for (unsigned int i=0;i<velems.size();i++)
            velems[i] = 0;
    }
    void setV(int vertex, int num, int index)
    {
        int bloc = vertex/BSIZE;
        int b_x  = vertex%BSIZE;
        velems[ bloc*BSIZE*nbElementPerVertex // start of the bloc
              + num*BSIZE                     // offset to the element
              + b_x                           // offset to the vertex
              ] = index+1;
    }

    void setE(int i, const Element& indices, Real vol, Real r, Real Kvol, Real Kr) //, const MaterialStiffness& K, const StrainDisplacement& /*J*/)
    {
        GPUElement& e = elems[i];
	for (unsigned int i=0;i<indices.size();i++)
	    e.v[i] = indices[i];
        e.initvol = vol;
	e.initr = r;
	e.Kvol = Kvol;
	e.Kr = Kr;
    }
};

} // namespace cuda

} // namespace gpu

} // namespace sofa

#endif
