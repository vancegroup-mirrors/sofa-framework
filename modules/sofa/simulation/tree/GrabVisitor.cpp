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
#include <sofa/simulation/tree/GrabVisitor.h>
#include <sofa/simulation/tree/GNode.h>


#include <sofa/component/mapping/LaparoscopicRigidMapping.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/core/componentmodel/collision/Contact.h>
#include <sofa/component/forcefield/PenalityContactForceField.h>


namespace sofa
{

namespace simulation
{

namespace tree
{
//#include <sofa/component/mapping/LaparoscopicRigidMapping.h>

  using namespace sofa::defaulttype;  
  using namespace sofa::component::mapping;
  using namespace sofa::component::forcefield;
  using namespace sofa::core::componentmodel::behavior;
  using namespace sofa::core::componentmodel::collision;
 
  
  void GrabVisitor::processGrab(GNode*, BaseMechanicalMapping* obj)
{
  if     ( LaparoscopicRigidMapping< MechanicalMapping< MechanicalState< LaparoscopicRigidTypes >, MechanicalState< RigidTypes > > > *tool = 
      dynamic_cast< LaparoscopicRigidMapping< MechanicalMapping< MechanicalState< LaparoscopicRigidTypes >, MechanicalState< RigidTypes > > > *>(obj))
  {    
    tool->grab();
  }
  else if(  LaparoscopicRigidMapping< core::Mapping< State<LaparoscopicRigidTypes>, MappedModel<RigidTypes> > > *tool = dynamic_cast< LaparoscopicRigidMapping< core::Mapping< State<LaparoscopicRigidTypes>, MappedModel<RigidTypes> > > *>(obj))
  {
    tool->grab();
  }
}
Visitor::Result GrabVisitor::processNodeTopDown(GNode* node)
{
   
	for_each(this, node, node->mechanicalMapping, &GrabVisitor::processGrab);
	return RESULT_CONTINUE;
}

//----------------------------------------------------------------------------------------------------------------
 
void GrabGetPointsVisitor::processGrabGetPoints(GNode*, InteractionForceField* obj)
{
  if (PenalityContactForceField<Vec3Types> *pff = dynamic_cast< PenalityContactForceField<Vec3Types> *>(obj)) 
  {
    pff->grabPoint( tool, points, collisions, triangle, index_point);
  }
}
Visitor::Result GrabGetPointsVisitor::processNodeTopDown(GNode* node)
{
  for_each(this, node, node->interactionForceField, &GrabGetPointsVisitor::processGrabGetPoints);
  return RESULT_CONTINUE;
}

//----------------------------------------------------------------------------------------------------------------
 
void GrabCollisionModelsVisitor::processGrabCollisionModels(GNode*, core::objectmodel::BaseObject* obj)
{
  if (Contact *c = dynamic_cast< Contact *>(obj))   
    c->getCorrespondingCollisionModels(ff,model1, model2);

}
Visitor::Result GrabCollisionModelsVisitor::processNodeTopDown(GNode* node)
{
  for_each(this, node, node->object, &GrabCollisionModelsVisitor::processGrabCollisionModels);
  return RESULT_CONTINUE;
}




    
} // namespace tree

} // namespace simulation

} // namespace sofa

