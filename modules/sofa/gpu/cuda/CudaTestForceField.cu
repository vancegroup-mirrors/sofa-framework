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
#include "CudaCommon.h"
#include "CudaMath.h"
#include "cuda.h"

#if defined(__cplusplus) && CUDA_VERSION != 2000
namespace sofa
{
namespace gpu
{
namespace cuda
{
#endif

extern "C"
{
void CudaTestForceField3f_addForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* f, const void* x, const void* v);
void CudaTestForceField3f_addDForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* df, const void* dx);
}

class __align__(16) GPUElement
{
public:
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

class __align__(16) GPUElementState
{
public:
    /// center
    CudaVec3<float> center;
    /// volume diff*Kvol
    float dv;
    float r,Kr;
    float dummy1,dummy2;
};

//////////////////////
// GPU-side methods //
//////////////////////

#define getX(i) (((const CudaVec3<float>*)x)[i])

__global__ void CudaTestForceField3f_calcForce_kernel(int nbElem, const GPUElement* elems, GPUElementState* state, const float* x)
{
    int index0 = umul24(blockIdx.x,BSIZE); //blockDim.x;
    int index1 = threadIdx.x;
    int index = index0+index1;

    GPUElement e = elems[index];

    GPUElementState s;

    if (index < nbElem)
    {
        s.center = getX(e.v[0]);
	s.center += getX(e.v[1]);
	s.center += getX(e.v[2]);
	s.center += getX(e.v[3]);
	s.center += getX(e.v[4]);
	s.center += getX(e.v[5]);
	s.center += getX(e.v[6]);
	s.center += getX(e.v[7]);
	s.center *= 0.125f;
	s.dv = 0;
	s.r = e.initr;
	s.Kr = e.Kr;
    }

    state[index] = s;

}

__global__ void CudaTestForceField3f_addForce_kernel(int nbVertex, unsigned int nbElemPerVertex, const GPUElement* elems, GPUElementState* state, const int* velems, float* f, const float* x)
{
    int index0 = umul24(blockIdx.x,BSIZE); //blockDim.x;
    int index1 = threadIdx.x;
    int index3 = umul24(index1,3); //3*index1;

        //! Dynamically allocated shared memory to reorder global memory access
    extern  __shared__  float temp[];

    // First copy x inside temp
    int iext = umul24(blockIdx.x,BSIZE*3)+index1; //index0*3+index1;

    temp[index1        ] = x[iext        ];
    temp[index1+  BSIZE] = x[iext+  BSIZE];
    temp[index1+2*BSIZE] = x[iext+2*BSIZE];

    __syncthreads();

    CudaVec3<float> pos1 = CudaVec3<float>::make(temp[index3  ],temp[index3+1],temp[index3+2]);

    CudaVec3<float> force = CudaVec3<float>::make(0.0f,0.0f,0.0f);

    velems+=index0*nbElemPerVertex+index1;

    if (index0+index1 < nbVertex)
    for (int s = 0;s < nbElemPerVertex; s++)
    {
        int i = *velems -1;
        velems+=BSIZE;
	if (i != -1)
	{
            int eindex = i >> 3; // element index
            i &= 7;              // vertice index inside the element
            //GPUElement e = elems[eindex];
            GPUElementState s = state[eindex];
	    CudaVec3<float> dp = pos1 - s.center;
	    float inv_r = invnorm(dp);
	    float r = __fdividef(1.0,inv_r);
	    float dr = (s.r-r)*s.Kr*inv_r;
	    force += dp*dr;

	}
    }

    __syncthreads();

    temp[index3  ] = force.x;
    temp[index3+1] = force.y;
    temp[index3+2] = force.z;

    __syncthreads();

    f[iext        ] += temp[index1        ];
    f[iext+  BSIZE] += temp[index1+  BSIZE];
    f[iext+2*BSIZE] += temp[index1+2*BSIZE];
}

__global__ void CudaTestForceField3f_calcDForce_kernel(int nbElem, const GPUElement* elems, GPUElementState* state, const float* x)
{
    int index0 = umul24(blockIdx.x,BSIZE); //blockDim.x;
    int index1 = threadIdx.x;
    int index = index0+index1;

    //GPUElement e = elems[index];
    GPUElementState s = state[index];

    if (index < nbElem)
    {
    }

    state[index] = s;

}

//////////////////////
// CPU-side methods //
//////////////////////

void CudaTestForceField3f_addForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* f, const void* x, const void* v)
{
    dim3 threads1(BSIZE,1);
    dim3 grid1((nbElem+BSIZE-1)/BSIZE,1);
    CudaTestForceField3f_calcForce_kernel<<< grid1, threads1>>>(nbElem, (const GPUElement*)elems, (GPUElementState*)state, (const float*)x);
    dim3 threads2(BSIZE,1);
    dim3 grid2((nbVertex+BSIZE-1)/BSIZE,1);
    CudaTestForceField3f_addForce_kernel<<< grid2, threads2, BSIZE*3*sizeof(float) >>>(nbVertex, nbElemPerVertex, (const GPUElement*)elems, (GPUElementState*)state, (const int*)velems, (float*)f, (const float*)x);
}

void CudaTestForceField3f_addDForce(unsigned int nbElem, unsigned int nbVertex, unsigned int nbElemPerVertex, const void* elems, void* state, const void* velems, void* df, const void* dx)
{
    dim3 threads1(BSIZE,1);
    dim3 grid1((nbElem+BSIZE-1)/BSIZE,1);
    //CudaTestForceField3f_calcDForce_kernel<<< grid1, threads1>>>(nbElem, (const GPUElement*)elems, (GPUElementState*)state, (const float*)dx);
    dim3 threads2(BSIZE,1);
    dim3 grid2((nbVertex+BSIZE-1)/BSIZE,1);
    //CudaTestForceField3f_addForce_kernel<<< grid2, threads2, BSIZE*3*sizeof(float) >>>(nbVertex, nbElemPerVertex, (const GPUElement*)elems, (GPUElementState*)state, (const int*)velems, (float*)df, (const float*)dx);
}

#if defined(__cplusplus) && CUDA_VERSION != 2000
} // namespace cuda
} // namespace gpu
} // namespace sofa
#endif
