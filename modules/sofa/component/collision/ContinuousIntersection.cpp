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
#include <sofa/helper/system/config.h>
#include <sofa/component/collision/ContinuousIntersection.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/collision/ContinuousTriangleIntersection.h>
#include <sofa/core/collision/Intersection.inl>
#include <iostream>
#include <algorithm>



namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace collision;

SOFA_DECL_CLASS(ContinuousIntersection)

int ContinuousIntersectionClass = core::RegisterObject("TODO-ContinuousIntersectionClass")
.add< ContinuousIntersection >()
;

ContinuousIntersection::ContinuousIntersection()
{
    intersectors.add<TriangleModel, TriangleModel, ContinuousIntersection>(this);
}

bool ContinuousIntersection::testIntersection(Triangle& t1, Triangle& t2)
{
    ContinuousTriangleIntersection intersectionT(t1, t2);
    return intersectionT.isCollision();
}

int ContinuousIntersection::computeIntersection(Triangle& t1, Triangle& t2, OutputVector* contacts)
{
    ContinuousTriangleIntersection intersectionT(t1, t2);
    //sout<<"Distance correction between Triangle - Triangle"<<sendl;
    core::collision::DetectionOutput* c = intersectionT.computeDetectionOutput(); // new DetectionOutput();
    if (c == NULL)
        return 0;
    else
    {
        contacts->push_back(*c);
        delete c;
        return 1;
    }
}

} // namespace collision

} // namespace component

} // namespace sofa

