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
#ifndef PLUGINS_PIM_POINTSONSURFACE_INL
#define PLUGINS_PIM_POINTSONSURFACE_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include "PointsOnSurface.h"
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>

namespace plugins
{

namespace pim
{

using namespace sofa::helper;
using namespace sofa::defaulttype;
using namespace sofa::core::objectmodel;

template <class DataTypes>
PointsOnSurface<DataTypes>::PointsOnSurface():
f_surfacePoints(initData(&f_surfacePoints, "surfacePoints", "") )
, f_setOfPoints(initData(&f_setOfPoints, "setOfPoints","") )
, f_outputPoints(initData(&f_outputPoints, "outputPoints","") )
, radius(initData(&radius, 0.3, "radius","") )
{
}

template <class DataTypes>
void PointsOnSurface<DataTypes>::init()
{
    addInput(&f_surfacePoints);
    addInput(&f_setOfPoints);
    addOutput(&f_outputPoints);
    setDirtyValue();
}

template <class DataTypes>
void PointsOnSurface<DataTypes>::reset()
{
}

template <class DataTypes>
void PointsOnSurface<DataTypes>::reinit()
{
}

template <class DataTypes>
void PointsOnSurface<DataTypes>::update()
{
    cleanDirty();

    const VecCoord& surfacePoints = f_surfacePoints.getValue();
    const VecCoord& setOfPoints = f_setOfPoints.getValue();

//    std::cout << "surfacePoints " << surfacePoints.size() << std::endl;
//    std::cout << "setOfPoints " << setOfPoints.size() << std::endl;

    for( unsigned i=0; i<surfacePoints.size(); ++i )
    {
        Real x=0.0,y=0.0,z=0.0;
        DataTypes::get(x,y,z,surfacePoints[i]);
        Vector3 sp = Vector3(x, y, z);
        for( unsigned j=0; j<setOfPoints.size(); ++j )
        {
            DataTypes::get(x,y,z,setOfPoints[j]);
            Vector3 result = Vector3(x, y, z) - sp;
            if (result.norm() < radius.getValue())
            {
                outSetPoints.insert(j);
//                break;
            }
        }
    }

//    std::cout << "outSetPoints " << outSetPoints.size() << std::endl;

    SetIndex& outputPoints = *(f_outputPoints.beginEdit());
    for( std::set<unsigned int>::const_iterator iter = outSetPoints.begin(); iter != outSetPoints.end(); ++iter )
    {
        outputPoints.push_back(*iter);
    }
    f_outputPoints.endEdit();
}

} // namespace pim

} // namespace plugins

#endif
