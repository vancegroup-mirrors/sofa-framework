#include <sofa/helper/system/config.h>
#include <sofa/component/collision/ContinuousIntersection.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/collision/ContinuousTriangleIntersection.h>
#include <sofa/core/componentmodel/collision/Intersection.inl>
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
    intersectors.add<TriangleModel, TriangleModel, ContinuousIntersection, false>(this);
}

bool ContinuousIntersection::testIntersection(Triangle& t1, Triangle& t2)
{
    ContinuousTriangleIntersection intersectionT(t1, t2);
    return intersectionT.isCollision();
}

int ContinuousIntersection::computeIntersection(Triangle& t1, Triangle& t2, OutputVector* contacts)
{
    ContinuousTriangleIntersection intersectionT(t1, t2);
    //std::cout<<"Distance correction between Triangle - Triangle"<<std::endl;
    core::componentmodel::collision::DetectionOutput* c = intersectionT.computeDetectionOutput(); // new DetectionOutput();
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

