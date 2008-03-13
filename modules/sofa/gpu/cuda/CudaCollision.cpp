#include "CudaDistanceGridCollisionModel.h"
#include "CudaSphereModel.h"
#include "CudaPointModel.h"
#include <sofa/component/collision/NewProximityIntersection.inl>
#include <sofa/component/collision/DiscreteIntersection.inl>
#include <sofa/component/collision/RayPickInteractor.h>
#include <sofa/component/collision/RayContact.h>
#include "CudaContactMapper.h"
#include <sofa/component/collision/BarycentricPenalityContact.inl>
#include <sofa/component/forcefield/PenalityContactForceField.h>
#include "CudaPenalityContactForceField.h"
#include <fstream>
#include <GL/gl.h>

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::gpu::cuda;

template <>
void BarycentricPenalityContact<CudaPointModel,CudaRigidDistanceGridCollisionModel,CudaVec3fTypes>::setDetectionOutputs(OutputVector* o)
{
    TOutputVector& outputs = *static_cast<TOutputVector*>(o);
    const bool printLog = this->f_printLog.getValue();
    if (ff==NULL)
    {
        MechanicalState1* mstate1 = mapper1.createMapping(model1);
        MechanicalState2* mstate2 = mapper2.createMapping(model2);
        ff = new ResponseForceField(mstate1,mstate2);
    }
    
    int insize = outputs.size();
    mapper1.setPoints1(&outputs);
    mapper2.setPoints2(&outputs);
    const double d0 = intersectionMethod->getContactDistance() + model1->getProximity() + model2->getProximity(); // - 0.001;
    int size = insize;
#if 0
    ff->clear(size);
    //int i = 0;
    for (int i=0; i<insize; i++)
    {
        int index = i; //oldIndex[i];
        if (index < 0) continue; // this contact is ignored
        //DetectionOutput* o = &outputs[i];
        //CollisionElement1 elem1(o->elem.first);
        //CollisionElement2 elem2(o->elem.second);
        //int index1 = elem1.getIndex();
        //int index2 = elem2.getIndex();
        int index1 = index;
        int index2 = index;
        //// Create mapping for first point
        //index1 = mapper1.addPoint(o->point[0], index1);
        //// Create mapping for second point
        //index2 = mapper2.addPoint(o->point[1], index2);
        double distance = d0 + outputs.get(i)->distance; // + mapper1.radius(elem1) + mapper2.radius(elem2);
        double stiffness = (model1->getContactStiffness(0) * model1->getContactStiffness(0))/distance;
        double mu_v = (model1->getContactFriction(0) + model1->getContactFriction(0));
        ff->addContact(index1, index2, outputs.get(i)->normal, (float)distance, (float)stiffness, (float)mu_v, (float)mu_v, index);
    }
#else
    double distance = d0; // + mapper1.radius(elem1) + mapper2.radius(elem2);
    double stiffness = (model1->getContactStiffness(0) * model1->getContactStiffness(0)); ///distance;
    ff->setContacts((float)distance, (float)stiffness, &outputs);
#endif
    // Update mappings
    mapper1.update();
    mapper2.update();
}

template <>
void BarycentricPenalityContact<CudaSphereModel,CudaRigidDistanceGridCollisionModel,CudaVec3fTypes>::setDetectionOutputs(OutputVector* o)
{
    TOutputVector& outputs = *static_cast<TOutputVector*>(o);
    const bool printLog = this->f_printLog.getValue();
    if (ff==NULL)
    {
        MechanicalState1* mstate1 = mapper1.createMapping(model1);
        MechanicalState2* mstate2 = mapper2.createMapping(model2);
        ff = new ResponseForceField(mstate1,mstate2);
    }
    
    int insize = outputs.size();
    mapper1.setPoints1(&outputs);
    mapper2.setPoints2(&outputs);
    const double d0 = intersectionMethod->getContactDistance() + model1->getProximity() + model2->getProximity(); // - 0.001;
    int size = insize;
#if 0
    ff->clear(size);
    //int i = 0;
    for (int i=0; i<insize; i++)
    {
        int index = i; //oldIndex[i];
        if (index < 0) continue; // this contact is ignored
        //DetectionOutput* o = &outputs[i];
        //CollisionElement1 elem1(o->elem.first);
        //CollisionElement2 elem2(o->elem.second);
        //int index1 = elem1.getIndex();
        //int index2 = elem2.getIndex();
        int index1 = index;
        int index2 = index;
        //// Create mapping for first point
        //index1 = mapper1.addPoint(o->point[0], index1);
        //// Create mapping for second point
        //index2 = mapper2.addPoint(o->point[1], index2);
        double distance = d0 + outputs.get(i)->distance; // + mapper1.radius(elem1) + mapper2.radius(elem2);
        double stiffness = (model1->getContactStiffness(0) * model1->getContactStiffness(0))/distance;
        double mu_v = (model1->getContactFriction(0) + model1->getContactFriction(0));
        ff->addContact(index1, index2, outputs.get(i)->normal, (float)distance, (float)stiffness, (float)mu_v, (float)mu_v, index);
    }
#else
    double distance = d0; // + mapper1.radius(elem1) + mapper2.radius(elem2);
    double stiffness = (model1->getContactStiffness(0) * model1->getContactStiffness(0)); ///distance;
    ff->setContacts((float)distance, (float)stiffness, &outputs);
#endif
    // Update mappings
    mapper1.update();
    mapper2.update();
}

} //namespace collision


} //namespace component


namespace gpu
{

namespace cuda
{

SOFA_DECL_CLASS(CudaCollision)

using namespace sofa::component::collision;

class CudaProximityIntersection : public sofa::component::collision::NewProximityIntersection
{
public:

    virtual void init()
    {
        sofa::component::collision::NewProximityIntersection::init();
	intersectors.add<CudaSphereModel, RayModel,          DiscreteIntersection, true>(this);
	intersectors.add<CudaSphereModel, RayPickInteractor, DiscreteIntersection, true>(this);
        //intersectors.add<CudaSphereModel, PointModel,        DiscreteIntersection, true>(this);
        intersectors.add<CudaSphereModel, CudaSphereModel,   DiscreteIntersection, false>(this);
        //intersectors.add<LineModel,       CudaSphereModel,   CudaProximityIntersection, true>(this);
        intersectors.add<TriangleModel,   CudaSphereModel,   CudaProximityIntersection, true>(this);
    }

};


int CudaProximityIntersectionClass = core::RegisterObject("TODO-CudaProximityIntersection")
.add< CudaProximityIntersection >()
;

sofa::helper::Creator<core::componentmodel::collision::Contact::Factory, component::collision::RayContact<CudaSphereModel> > RayCudaSphereContactClass("default",true);
sofa::helper::Creator<core::componentmodel::collision::Contact::Factory, component::collision::RayContact<CudaSphereModel> > RayCudaSphereContactClass2("LagrangianMultiplier",true);
sofa::helper::Creator<core::componentmodel::collision::Contact::Factory, component::collision::RayContact<CudaSphereModel> > RayCudaSphereContactClass3("FrictionContact",true);


//sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaRigidDistanceGridCollisionModel, CudaRigidDistanceGridCollisionModel,CudaVec3fTypes> > CudaDistanceGridCudaDistanceGridContactClass("default", true);
sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaPointModel, CudaRigidDistanceGridCollisionModel,CudaVec3fTypes> > CudaPointCudaDistanceGridContactClass("default", true);
sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaSphereModel, CudaRigidDistanceGridCollisionModel,CudaVec3fTypes> > CudaSphereCudaDistanceGridContactClass("default", true);
//sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaRigidDistanceGridCollisionModel, sofa::component::collision::RigidDistanceGridCollisionModel> > CudaDistanceGridDistanceGridContactClass("default", true);
//sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaRigidDistanceGridCollisionModel, sofa::component::collision::PointModel> > CudaDistanceGridPointContactClass("default", true);
//sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaRigidDistanceGridCollisionModel, sofa::component::collision::SphereModel> > CudaDistanceGridSphereContactClass("default", true);
//sofa::helper::Creator<sofa::core::componentmodel::collision::Contact::Factory, sofa::component::collision::BarycentricPenalityContact<CudaRigidDistanceGridCollisionModel, sofa::component::collision::TriangleModel> > CudaDistanceGridTriangleContactClass("default", true);


} // namespace cuda

} // namespace gpu

} // namespace sofa
