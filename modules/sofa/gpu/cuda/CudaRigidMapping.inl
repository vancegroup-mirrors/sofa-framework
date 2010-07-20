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
#ifndef SOFA_GPU_CUDA_CUDARIGIDMAPPING_INL
#define SOFA_GPU_CUDA_CUDARIGIDMAPPING_INL

#include "CudaRigidMapping.h"
#include <sofa/component/mapping/RigidMapping.inl>
#include <sofa/helper/accessor.h>

namespace sofa
{

namespace gpu
{

namespace cuda
{

using sofa::defaulttype::Mat3x3f;
using sofa::defaulttype::Vec3f;

extern "C"
{
void RigidMappingCuda3f_apply(unsigned int size, const Mat3x3f& rotation, const Vec3f& translation, void* out, void* rotated, const void* in);
void RigidMappingCuda3f_applyJ(unsigned int size, const Vec3f& v, const Vec3f& omega, void* out, const void* rotated);
void RigidMappingCuda3f_applyJT(unsigned int size, unsigned int nbloc, void* out, const void* rotated, const void* in);
}

} // namespace cuda

} // namespace gpu

namespace component
{

namespace mapping
{

using namespace gpu::cuda;

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaRigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaRigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<gpu::cuda::CudaRigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    int nbloc = ((points.size()+BSIZE-1)/BSIZE);
    if (nbloc > 512) nbloc = 512;
    data.tmp.recreate(2*nbloc);
    RigidMappingCuda3f_applyJT(points.size(), nbloc, data.tmp.deviceWrite(), rotatedPoints.deviceRead(), in.deviceRead());
    helper::ReadAccessor<gpu::cuda::CudaVec3fTypes::VecDeriv> tmp = data.tmp;
    for(int i=0;i<nbloc;i++)
    {
        v += tmp[2*i];
        omega += tmp[2*i+1];
    }
    out[index.getValue()].getVCenter() += v;
    out[index.getValue()].getVOrientation() += omega;
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaRigid3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<gpu::cuda::CudaRigid3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}

//////// Rigid3d ////////

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3dTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3dTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3dTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    int nbloc = ((points.size()+BSIZE-1)/BSIZE);
    if (nbloc > 512) nbloc = 512;
    data.tmp.recreate(2*nbloc);
    RigidMappingCuda3f_applyJT(points.size(), nbloc, data.tmp.deviceWrite(), rotatedPoints.deviceRead(), in.deviceRead());
    helper::ReadAccessor<gpu::cuda::CudaVec3fTypes::VecDeriv> tmp = data.tmp;
    for(int i=0;i<nbloc;i++)
    {
        v += tmp[2*i];
        omega += tmp[2*i+1];
    }
    out[index.getValue()].getVCenter() += v;
    out[index.getValue()].getVOrientation() += omega;
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<defaulttype::Rigid3dTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<defaulttype::Rigid3dTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}


//////// Rigid3f ////////

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}

template <>
void RigidMapping<sofa::core::behavior::MechanicalMapping< sofa::core::behavior::MechanicalState<defaulttype::Rigid3fTypes>, sofa::core::behavior::MechanicalState<gpu::cuda::CudaVec3fTypes> > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    int nbloc = ((points.size()+BSIZE-1)/BSIZE);
    if (nbloc > 512) nbloc = 512;
    data.tmp.recreate(2*nbloc);
    RigidMappingCuda3f_applyJT(points.size(), nbloc, data.tmp.deviceWrite(), rotatedPoints.deviceRead(), in.deviceRead());
    helper::ReadAccessor<gpu::cuda::CudaVec3fTypes::VecDeriv> tmp = data.tmp;
    for(int i=0;i<nbloc;i++)
    {
        v += tmp[2*i];
        omega += tmp[2*i+1];
    }
    out[index.getValue()].getVCenter() += v;
    out[index.getValue()].getVOrientation() += omega;
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<defaulttype::Rigid3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::apply( Out::VecCoord& out, const In::VecCoord& in )
{
    const VecCoord& points = this->points.getValue();
    Coord translation;
    Mat rotation;
    rotatedPoints.resize(points.size());
    out.recreate(points.size());

    translation = in[index.getValue()].getCenter();
    in[index.getValue()].writeRotationMatrix(rotation);

    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    rotatedPoints[i] = rotation*points[i];
    //    out[i] = rotatedPoints[i];
    //    out[i] += translation;
    //}
    RigidMappingCuda3f_apply(points.size(), rotation, translation, out.deviceWrite(), rotatedPoints.deviceWrite(), points.deviceRead());
}

template <>
void RigidMapping<sofa::core::Mapping< sofa::core::behavior::State<defaulttype::Rigid3fTypes>, sofa::core::behavior::MappedModel<gpu::cuda::CudaVec3fTypes> > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in )
{
    const VecCoord& points = this->points.getValue();
    Deriv v,omega;
    out.recreate(points.size());
    v = in[index.getValue()].getVCenter();
    omega = in[index.getValue()].getVOrientation();
    //for(unsigned int i=0;i<points.size();i++)
    //{
    //    // out = J in
    //    // J = [ I -OM^ ]
    //    out[i] =  v - cross(rotatedPoints[i],omega);
    //}
    RigidMappingCuda3f_applyJ(points.size(), v, omega, out.deviceWrite(), rotatedPoints.deviceRead());
}


} // namespace mapping

} // namespace component

} // namespace sofa

#endif
