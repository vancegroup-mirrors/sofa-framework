#ifndef SOFA_COMPONENT_COLLISION_EDGEREMOVECONTACT_H
#define SOFA_COMPONENT_COLLISION_EDGEREMOVECONTACT_H

#include <sofa/core/componentmodel/collision/Contact.h>
#include <sofa/helper/Factory.h>
#include "LineModel.h"
#include "SphereModel.h"

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;

class EdgeRemoveContact : public core::componentmodel::collision::Contact
{
public:
    typedef LineModel CollisionModel1;
    typedef SphereModel CollisionModel2;
    typedef core::componentmodel::collision::Intersection Intersection;
    typedef core::componentmodel::collision::DetectionOutputVector OutputVector;
    typedef core::componentmodel::collision::TDetectionOutputVector<CollisionModel1,CollisionModel2> TOutputVector;

protected:
    CollisionModel1* model1;
    CollisionModel2* model2;
    TOutputVector collisions;
    
public:
    EdgeRemoveContact(CollisionModel1* model1, CollisionModel2* model2, core::componentmodel::collision::Intersection* /*intersectionMethod*/)
    : model1(model1), model2(model2)
    {
    }

    ~EdgeRemoveContact()
    {
    }
    
    std::pair<core::CollisionModel*,core::CollisionModel*> getCollisionModels() { return std::make_pair(model1,model2); }

    void setDetectionOutputs(OutputVector* outputs)
    {
        collisions = *static_cast<TOutputVector*>(outputs);
    }
    
    const TOutputVector& getDetectionOutputs() const { return collisions; }
    
    void createResponse(core::objectmodel::BaseContext* group);
    
    void removeResponse()
    {
    }
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
