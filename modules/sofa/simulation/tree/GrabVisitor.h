/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_SIMULATION_TREE_GRABACTION_H
#define SOFA_SIMULATION_TREE_GRABACTION_H

#include <sofa/simulation/tree/Visitor.h>
#include <sofa/core/componentmodel/behavior/BaseMechanicalMapping.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

class GrabVisitor : public Visitor
{ 
public:
        void processGrab(GNode* node, sofa::core::componentmodel::behavior::BaseMechanicalMapping* obj);
	
	virtual Result processNodeTopDown(GNode* node);

	/// Return a category name for this action.
	/// Only used for debugging / profiling purposes
	virtual const char* getCategoryName() const { return "grab"; }

	/// Specify whether this action can be parallelized.
	virtual bool isThreadSafe() const { return true; }
};

class GrabGetPointsVisitor : public Visitor
{
  public:
    //IN
    core::componentmodel::behavior::MechanicalState<defaulttype::Vec3Types> *tool; //mechanical state of the tool
    helper::vector< unsigned int > points;  //list of grabing points in the tool
    
    //OUT
    helper::vector< unsigned int > triangle; //index of the triangles used for grabing
    helper::vector< unsigned int > index_point;    
    helper::vector< std::pair< core::objectmodel::BaseObject*, defaulttype::Vec3f> > collisions; //pair of forcefield and point ( coordinates of a point to link with the tool)
    
    void processGrabGetPoints(GNode* node, sofa::core::componentmodel::behavior::InteractionForceField* obj);
	
    virtual Result processNodeTopDown(GNode* node);

	/// Return a category name for this action.
	/// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const { return "grabgetpoints"; }

	/// Specify whether this action can be parallelized.
    virtual bool isThreadSafe() const { return true; }
};

class GrabCollisionModelsVisitor : public Visitor
{ 
  public:
    sofa::core::componentmodel::behavior::InteractionForceField* ff;
    core::CollisionModel *model1, *model2;
    
    void processGrabCollisionModels(GNode* node, core::objectmodel::BaseObject* obj);
	
    virtual Result processNodeTopDown(GNode* node);

	/// Return a category name for this action.
	/// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const { return "grabcollisionmodels"; }

	/// Specify whether this action can be parallelized.
    virtual bool isThreadSafe() const { return true; }
};

} // namespace tree

} // namespace simulation

} // namespace sofa

#endif
