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
#ifndef PLUGINS_PIM_POINTSONSURFACE_H
#define PLUGINS_PIM_POINTSONSURFACE_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/component/topology/PointSubset.h>

namespace plugins
{

namespace pim
{

using namespace sofa::core::componentmodel::behavior;
using namespace sofa::core::componentmodel::topology;
using namespace sofa::core::objectmodel;

/**
 *
 */
template <class DataTypes>
class PointsOnSurface : public virtual sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PointsOnSurface,DataTypes),sofa::core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Real Real;
    typedef sofa::component::topology::PointSubset SetIndex;

public:

    PointsOnSurface();

    ~PointsOnSurface(){}

    void init();

    void reinit();

    void reset();

    void update();

    Data<VecCoord> f_surfacePoints;
    Data<VecCoord> f_setOfPoints;
    Data<SetIndex> f_outputPoints;
    Data<double> radius;
    std::set <unsigned int> outSetPoints; 

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const PointsOnSurface<DataTypes>* = NULL)
    {
      return DataTypes::Name();
    }

};

#if defined(WIN32) && !defined(SOFA_COMPONENT_ENGINE_PROGRESSIVESCALING_CPP)
#pragma warning(disable : 4231)
#ifndef SOFA_FLOAT
template class SOFA_COMPONENT_ENGINE_API PointsOnSurface<defaulttype::Vec3dTypes>;
template class SOFA_COMPONENT_ENGINE_API PointsOnSurface<defaulttype::Rigid3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class SOFA_COMPONENT_ENGINE_API PointsOnSurface<defaulttype::Vec3fTypes>;
template class SOFA_COMPONENT_ENGINE_API PointsOnSurface<defaulttype::Rigid3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace pim

} // namespace plugins

#endif
