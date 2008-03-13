#ifndef SOFA_COMPONENT_COLLISION_CONTINUOUSINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_CONTINUOUSINTERSECTION_H

#include <sofa/component/collision/DiscreteIntersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <sofa/component/collision/Sphere.h>
#include <sofa/component/collision/Triangle.h>
#include <sofa/component/collision/Cube.h>
#include <sofa/component/collision/Ray.h>

namespace sofa
{

namespace component
{

namespace collision
{

class ContinuousIntersection : public DiscreteIntersection
{
public:
    ContinuousIntersection();

    /// returns true if algorithm uses continous detection
    virtual bool useContinuous() const { return true; }

    bool testIntersection(Triangle& ,Triangle&);

    int computeIntersection(Triangle& ,Triangle&, OutputVector*);

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
