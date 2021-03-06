/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_COLLISION_MESHNEWPROXIMITYINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_MESHNEWPROXIMITYINTERSECTION_H

#include <sofa/component/collision/NewProximityIntersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <sofa/component/collision/CapsuleModel.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/collision/LineModel.h>
#include <sofa/component/collision/PointModel.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/component/collision/MeshIntTool.h>
#include <sofa/component/collision/IntrUtility3.h>
#include <sofa/component/collision/BaseIntTool.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API MeshNewProximityIntersection : public core::collision::BaseIntersector
{
    typedef NewProximityIntersection::OutputVector OutputVector;

public:
    MeshNewProximityIntersection(NewProximityIntersection* object, bool addSelf=true);


    template <class T1,class T2>
    bool testIntersection(T1 & e1,T2 & e2){
        return BaseIntTool::testIntersection(e1,e2,intersection->getAlarmDistance());
    }


    int computeIntersection(Point&, Point&, OutputVector*);
    template <class T> int computeIntersection(TSphere<T>&, Point&, OutputVector*);
    int computeIntersection(Line&, Point&, OutputVector*);
    template <class T> int computeIntersection(Line&, TSphere<T>&, OutputVector*);
    int computeIntersection(Line&, Line&, OutputVector*);
    int computeIntersection(Triangle&, Point&, OutputVector*);

    template <class T> int computeIntersection(Triangle&, TSphere<T>&, OutputVector*);
    int computeIntersection(Triangle&, Line&, OutputVector*);

    int computeIntersection(Triangle&, Triangle&, OutputVector*);

    template <class T1,class T2>
    int computeIntersection(T1 & e1,T2 & e2,OutputVector* contacts){
        return MeshIntTool::computeIntersection(e1,e2,e1.getProximity() + e2.getProximity() + intersection->getAlarmDistance(),e1.getProximity() + e2.getProximity() + intersection->getContactDistance(),contacts);
    }

    static inline int doIntersectionLineLine(SReal dist2, const Vector3& p1, const Vector3& p2, const Vector3& q1, const Vector3& q2, OutputVector* contacts, int id);

    static inline int doIntersectionLinePoint(SReal dist2, const Vector3& p1, const Vector3& p2, const Vector3& q, OutputVector* contacts, int id, bool swapElems = false);

    static inline int doIntersectionTrianglePoint(SReal dist2, int flags, const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& n, const Vector3& q, OutputVector* contacts, int id, bool swapElems = false);

    static inline int doIntersectionTrianglePoint2(SReal dist2, int flags, const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& n, const Vector3& q, OutputVector* contacts, int id, bool swapElems = false);

protected:

    NewProximityIntersection* intersection;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
