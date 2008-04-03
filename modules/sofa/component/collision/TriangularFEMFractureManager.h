#ifndef SOFA_COMPONENT_COLLISION_TRIANGULARFEMFRACTUREMANAGER_H
#define SOFA_COMPONENT_COLLISION_TRIANGULARFEMFRACTUREMANAGER_H

#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/componentmodel/collision/Intersection.h>
#include <sofa/core/componentmodel/collision/NarrowPhaseDetection.h>
#include <sofa/component/collision/SharpLineModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/topology/PointSubset.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/tree/AnimateBeginEvent.h>
#include <sofa/simulation/tree/AnimateEndEvent.h>
#include <sofa/core/componentmodel/behavior/BaseForceField.h>

#include <fstream>

namespace sofa
{

namespace component
{

namespace collision
{

class TriangularFEMFractureManager : public core::objectmodel::BaseObject
{
protected:
	core::componentmodel::behavior::BaseForceField *femModel;

public:
    TriangularFEMFractureManager();

    virtual ~TriangularFEMFractureManager();

    virtual void init();

    virtual void reset();
    
    virtual void handleEvent(sofa::core::objectmodel::Event* event);
    
    virtual void doFracture();

	Data<std::string> f_femName;

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
