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
#include <sofa/component/collision/TetrahedronDiscreteIntersection.h>
#include <sofa/helper/system/config.h>
#include <sofa/helper/FnDispatcher.inl>
#include <sofa/component/collision/DiscreteIntersection.inl>
#include <sofa/core/collision/Intersection.inl>
//#include <sofa/component/collision/ProximityIntersection.h>
#include <sofa/helper/proximity.h>
#include <iostream>
#include <algorithm>
#include <sofa/core/collision/IntersectorFactory.h>


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace sofa::core::collision;

SOFA_DECL_CLASS(TetrahedronDiscreteIntersection)

IntersectorCreator<DiscreteIntersection, TetrahedronDiscreteIntersection> TetrahedronDiscreteIntersectors("Ray");

TetrahedronDiscreteIntersection::TetrahedronDiscreteIntersection(DiscreteIntersection* object)
    : intersection(object)
{
    intersection->intersectors.add<TetrahedronModel, PointModel,       TetrahedronDiscreteIntersection>  (this);
}

bool TetrahedronDiscreteIntersection::testIntersection(Tetrahedron&, Point&)
{
    return true;
}

int TetrahedronDiscreteIntersection::computeIntersection(Tetrahedron& e1, Point& e2, OutputVector* contacts)
{
    Vector3 n = e2.n();
    if (n == Vector3()) return 0; // no normal on point -> either an internal points or normal is not available

    if (e1.getCollisionModel()->getMechanicalState() == e2.getCollisionModel()->getMechanicalState())
    {
        // self-collisions: make sure the point is not one of the vertices of the tetrahedron
        int i = e2.getIndex();
        if (i == e1.p1Index() || i == e1.p2Index() || i == e1.p3Index() || i == e1.p4Index())
            return 0;
    }

    Vector3 P = e2.p();
    Vector3 b0 = e1.getBary(P);
    if (b0[0] < 0 || b0[1] < 0 || b0[2] < 0 || (b0[0]+b0[1]+b0[2]) > 1)
        return 1; // out of tetrahedron

    // Find the point on the surface of the tetrahedron in the direction of -n
    Vector3 bdir = e1.getDBary(-n);
    //sout << "b0 = "<<b0<<" \tbdir = "<<bdir<<sendl;
    double l1 = 1.0e10;
    for (int c=0; c<3; ++c)
    {
        if (bdir[c] < -1.0e-10)
        {
            double l = -b0[c]/bdir[c];
            if (l < l1) l1 = l;
        }
    }
    // 4th plane : bx+by+bz = 1
    {
        double bd = bdir[0]+bdir[1]+bdir[2];
        if (bd > 1.0e-10)
        {
            double l = (1-(b0[0]+b0[1]+b0[2]))/bd;
            if (l < l1) l1 = l;
        }
    }
    if (l1 >= 1.0e9) l1 = 0;
    double l = l1;
    Vector3 X = P-n*l;

    contacts->resize(contacts->size()+1);
    DetectionOutput *detection = &*(contacts->end()-1);
    detection->point[0] = X;
    detection->point[1] = P;
    detection->normal = -n;
    detection->value = -l;
    detection->elem.first = e1;
    detection->elem.second = e2;
    detection->id = e2.getIndex();
    return 1;
}

} // namespace collision

} // namespace component

} // namespace sofa
