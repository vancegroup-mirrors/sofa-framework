#ifndef SOFA_COMPONENT_COLLISION_CARVINGMANAGER_H
#define SOFA_COMPONENT_COLLISION_CARVINGMANAGER_H

#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/componentmodel/collision/Intersection.h>
#include <sofa/core/componentmodel/collision/NarrowPhaseDetection.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/topology/PointSubset.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>

#include <fstream>

namespace sofa
{

namespace component
{

namespace collision
{

class CarvingManager : public core::objectmodel::BaseObject
{
public:
    typedef TriangleSetModel::DataTypes DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    
    typedef SphereModel ToolModel;
    typedef helper::vector<core::componentmodel::collision::DetectionOutput> ContactVector;

    Data < std::string > f_modelTool;
    Data < std::string > f_modelSurface;
    Data < Real > f_minDistance;
    Data < Real > f_maxDistance;
    Data < Real > f_edgeDistance;
    
    Data < bool > active;
    Data < char > keyEvent;
    Data < char > keySwitchEvent;

protected:
    ToolModel* modelTool;
    TriangleSetModel* modelSurface;
    core::componentmodel::collision::Intersection* intersectionMethod;
    core::componentmodel::collision::NarrowPhaseDetection* detectionNP;

public:
    CarvingManager();

    virtual ~CarvingManager();

    virtual void init();

    virtual void reset();
    
    virtual void handleEvent(sofa::core::objectmodel::Event* event);
    
    virtual void doCarve();

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
