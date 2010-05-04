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
#ifndef SOFA_GPU_CUDA_CUDACONTACTMAPPER_H
#define SOFA_GPU_CUDA_CUDACONTACTMAPPER_H

#include <sofa/component/collision/BarycentricContactMapper.h>
#include <sofa/component/collision/RigidContactMapper.inl>
#include <sofa/component/collision/SubsetContactMapper.inl>
#include <sofa/gpu/cuda/CudaDistanceGridCollisionModel.h>
#include <sofa/gpu/cuda/CudaPointModel.h>
#include <sofa/gpu/cuda/CudaSphereModel.h>
#include <sofa/gpu/cuda/CudaCollisionDetection.h>
#include <sofa/gpu/cuda/CudaRigidMapping.h>
#include <sofa/gpu/cuda/CudaSubsetMapping.h>


namespace sofa
{

namespace gpu
{

namespace cuda
{

extern "C"
{
void RigidContactMapperCuda3f_setPoints2(unsigned int size, unsigned int nbTests, unsigned int maxPoints, const void* tests, const void* contacts, void* map);
void SubsetContactMapperCuda3f_setPoints1(unsigned int size, unsigned int nbTests, unsigned int maxPoints, unsigned int nbPointsPerElem, const void* tests, const void* contacts, void* map);
}

} // namespace cuda

} // namespace gpu

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using sofa::core::componentmodel::collision::GPUDetectionOutputVector;


/// Mapper for CudaRigidDistanceGridCollisionModel
template <class DataTypes>
class ContactMapper<sofa::gpu::cuda::CudaRigidDistanceGridCollisionModel,DataTypes> : public RigidContactMapper<sofa::gpu::cuda::CudaRigidDistanceGridCollisionModel,DataTypes>
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef RigidContactMapper<sofa::gpu::cuda::CudaRigidDistanceGridCollisionModel,DataTypes> Inherit;
    typedef typename Inherit::MMechanicalState MMechanicalState;
    typedef typename Inherit::MCollisionModel MCollisionModel;
    
    int addPoint(const Coord& P, int index, Real& r)
    {
        int i = this->Inherit::addPoint(P, index, r);
        if (!this->mapping)
        {
            MCollisionModel* model = this->model;
            MMechanicalState* outmodel = this->outmodel;
            typename DataTypes::Coord& x = (*outmodel->getX())[i];
            typename DataTypes::Deriv& v = (*outmodel->getV())[i];
            if (model->isTransformed(index))
            {
                x = model->getTranslation(index) + model->getRotation(index) * P;
            }
            else
            {
                x = P;
            }
            v = typename DataTypes::Deriv();
        }
        return i;
    }

    void setPoints2(GPUDetectionOutputVector* outputs)
    {
        int n = outputs->size();
        int nt = outputs->nbTests();
        int maxp = 0;
        for (int i=0;i<nt;i++)
            if (outputs->rtest(i).curSize > maxp) maxp = outputs->rtest(i).curSize;
        if (this->outmodel)
            this->outmodel->resize(n);
        if (this->mapping)
        {
            this->mapping->points.beginEdit()->fastResize(n);
            this->mapping->rotatedPoints.fastResize(n);
            RigidContactMapperCuda3f_setPoints2(n, nt, maxp, outputs->tests.deviceRead(), outputs->results.deviceRead(), this->mapping->points.beginEdit()->deviceWrite());
        }
        else
        {
            RigidContactMapperCuda3f_setPoints2(n, nt, maxp, outputs->tests.deviceRead(), outputs->results.deviceRead(), this->outmodel->getX()->deviceWrite());
        }
    }
};


/// Mapper for CudaPointDistanceGridCollisionModel
template <class DataTypes>
class ContactMapper<sofa::gpu::cuda::CudaPointModel,DataTypes> : public SubsetContactMapper<sofa::gpu::cuda::CudaPointModel,DataTypes>
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef SubsetContactMapper<sofa::gpu::cuda::CudaPointModel,DataTypes> Inherit;
    typedef typename Inherit::MMechanicalState MMechanicalState;
    typedef typename Inherit::MCollisionModel MCollisionModel;
    
    int addPoint(const Coord& P, int index, Real& r)
    {
        int i = Inherit::addPoint(P, index, r);
        return i;
    }

    void setPoints1(GPUDetectionOutputVector* outputs)
    {
        int n = outputs->size();
        int nt = outputs->nbTests();
        int maxp = 0;
        for (int i=0;i<nt;i++)
            if (outputs->rtest(i).curSize > maxp) maxp = outputs->rtest(i).curSize;
        this->mapping->data.map.fastResize(n);
        SubsetContactMapperCuda3f_setPoints1(n, nt, maxp, this->model->groupSize.getValue(), outputs->tests.deviceRead(), outputs->results.deviceRead(), this->mapping->data.map.deviceWrite());
    }
};


template <class DataTypes>
class ContactMapper<sofa::gpu::cuda::CudaSphereModel,DataTypes> : public SubsetContactMapper<sofa::gpu::cuda::CudaSphereModel,DataTypes>
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef SubsetContactMapper<sofa::gpu::cuda::CudaSphereModel,DataTypes> Inherit;
    typedef typename Inherit::MMechanicalState MMechanicalState;
    typedef typename Inherit::MCollisionModel MCollisionModel;
    
    int addPoint(const Coord& P, int index, Real& r)
    {
        int i = this->Inherit::addPoint(P, index, r);
        return i;
    }

    void setPoints1(GPUDetectionOutputVector* outputs)
    {
        int n = outputs->size();
        int nt = outputs->nbTests();
        int maxp = 0;
        for (int i=0;i<nt;i++)
            if (outputs->rtest(i).curSize > maxp) maxp = outputs->rtest(i).curSize;
        this->mapping->data.map.fastResize(n);
        SubsetContactMapperCuda3f_setPoints1(n, nt, maxp, 0, outputs->tests.deviceRead(), outputs->results.deviceRead(), this->mapping->data.map.deviceWrite());
    }
};



} // namespace collision

} // namespace component

} // namespace sofa

#endif
