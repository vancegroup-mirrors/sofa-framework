/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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

#include <sofa/core/componentmodel/behavior/BaseController.h>

#include <fstream>

#include <sofa/component/topology/TetrahedronSetTopologyModifier.h>

namespace sofa
{

namespace component
{

namespace collision
{

class CarvingManager : public core::componentmodel::behavior::BaseController
{
public:
    typedef TriangleModel::DataTypes DataTypes;
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

	sofa::core::componentmodel::topology::BaseMeshTopology* _topology;
	sofa::component::topology::TetrahedronSetTopologyModifier* tetraMod; 

protected:
    ToolModel* modelTool;
    TriangleModel* modelSurface;
    sofa::core::componentmodel::topology::TopologicalMapping * topoMapping;
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
