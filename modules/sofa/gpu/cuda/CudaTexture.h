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
#ifndef CUDATEXTURE_H
#define CUDATEXTURE_H

#include "CudaMath.h"

/// Accesss to a vector in global memory using either direct access or linear texture
/// TIn is the data type as stored in memory
/// TOut is the data type as requested
template<class TIn, class TOut, bool useTexture>
class InputVector;

/// Direct access to a vector in global memory
/// TIn is the data type as stored in memory
/// TOut is the data type as requested
template<class TIn, class TOut>
class InputVectorDirect
{
public:
    __host__ void set(const TIn*) {}
    __inline__ __device__ TOut get(int i, const TIn* x) { return TOut::make(x[i]); }
};

/// Linear texture access to a vector in global memory
/// TIn is the data type as stored in memory
/// TOut is the data type as requested
template<class TIn, class TOut>
class InputVectorTexture;

template<class TIn, class TOut>
class InputVector<TIn, TOut, false> : public InputVectorDirect<TIn, TOut>
{
};

template<class TIn, class TOut>
class InputVector<TIn, TOut, true> : public InputVectorTexture<TIn, TOut>
{
};

template<class TOut>
class InputVectorTexture<float, TOut>
{
public:
    typedef float TIn;
    static texture<float,1,cudaReadModeElementType> tex;

    __host__ void set(const TIn* x)
    {
	static const void* cur = NULL;
	if (x != cur)
	{
	    cudaBindTexture((size_t*)NULL, tex, x);
	    cur = x;
	}
    }
    __inline__ __device__ TOut get(int i, const TIn* x)
    {
	return tex1Dfetch(tex, i);
    }
};

template<class TOut>
class InputVectorTexture<CudaVec2<float>, TOut>
{
public:
    typedef CudaVec2<float> TIn;
    static texture<float2,1,cudaReadModeElementType> tex;

    __host__ void set(const TIn* x)
    {
	static const void* cur = NULL;
	if (x != cur)
	{
	    cudaBindTexture((size_t*)NULL, tex, x);
	    cur = x;
	}
    }
    __inline__ __device__ TOut get(int i, const TIn* x)
    {
	return TOut::make(tex1Dfetch(tex, i));
    }
};

template<class TOut>
class InputVectorTexture<CudaVec3<float>, TOut>
{
public:
    typedef CudaVec3<float> TIn;
    static texture<float,1,cudaReadModeElementType> tex;

    __host__ void set(const TIn* x)
    {
	static const void* cur = NULL;
	if (x != cur)
	{
	    cudaBindTexture((size_t*)NULL, tex, x);
	    cur = x;
	}
    }
    __inline__ __device__ TOut get(int i, const TIn* x)
    {
	int i3 = umul24(i,3);
	float x1 = tex1Dfetch(tex, i3);
	float x2 = tex1Dfetch(tex, i3+1);
	float x3 = tex1Dfetch(tex, i3+2);
	return TOut::make(x1,x2,x3);
    }
};

template<class TOut>
class InputVectorTexture<CudaVec4<float>, TOut>
{
public:
    typedef CudaVec4<float> TIn;
    static texture<float4,1,cudaReadModeElementType> tex;

    __host__ void set(const TIn* x)
    {
	static const void* cur = NULL;
	if (x != cur)
	{
	    cudaBindTexture((size_t*)NULL, tex, x);
	    cur = x;
	}
    }
    __inline__ __device__ TOut get(int i, const TIn* x)
    {
	return TOut::make(tex1Dfetch(tex, i));
    }
};



#endif
