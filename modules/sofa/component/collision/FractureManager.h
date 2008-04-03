#ifndef SOFA_COMPONENT_COLLISION_FRACTUREMANAGER_H
#define SOFA_COMPONENT_COLLISION_FRACTUREMANAGER_H

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

#include <fstream>

namespace sofa
{

namespace component
{

namespace collision
{

class FracturingPoint : public virtual core::objectmodel::BaseObject, public virtual core::VisualModel
{
public:
    typedef TriangleSetModel::DataTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    Coord lastPos, pos, newPos;
    int toolElemIndex;
    topology::PointSubset points;
    DataPtr<topology::PointSubset> f_points;
    core::componentmodel::behavior::MechanicalState<DataTypes>* mstate;
	FracturingPoint()
    : toolElemIndex(-1)
    , f_points( initDataPtr(&f_points, &points, "points", "cut-related points") )
    , mstate(NULL)
    {
    }
    
    virtual void init()
    {
        this->core::objectmodel::BaseObject::init();
        mstate = getContext()->get<core::componentmodel::behavior::MechanicalState<DataTypes> >();
    }
    
    // Handle topological changes
    virtual void handleTopologyChange()
    {
        if (!mstate) return;
	sofa::core::componentmodel::topology::BaseTopology *topology = getContext()->get<sofa::core::componentmodel::topology::BaseTopology>();
        
	std::list<const sofa::core::componentmodel::topology::TopologyChange *>::const_iterator itBegin=topology->firstChange();
	std::list<const sofa::core::componentmodel::topology::TopologyChange *>::const_iterator itEnd=topology->lastChange();
        
	points.handleTopologyEvents(itBegin,itEnd,mstate->getSize());
    }
    
    // -- VisualModel interface
    
    virtual void draw();    
    void initTextures() { }
    void update() { }
};

class FractureManager : public core::objectmodel::BaseObject
{
public:
    typedef TriangleModel::Coord Coord;
    typedef Coord::value_type Real;

    sofa::helper::vector<FracturingPoint*> fracturingPoints;
	bool fracturePointDetected;
    Data < std::string > f_modelSurface;
	Data < Real > min_elongation;


protected:
    TriangleModel* modelSurface;

public:
    FractureManager();

    virtual ~FractureManager();

    virtual void init();

    virtual void reset();
    
    virtual void handleEvent(sofa::core::objectmodel::Event* event);
    
    virtual void doFracture();

	int getFractureEdgeFromPoint(int p, Real& alpha);

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
