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
#include "PipelineImpl.h"
#include <sofa/component/collision/DiscreteIntersection.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

using namespace sofa::core;
using namespace sofa::core::objectmodel;
using namespace sofa::core::componentmodel::behavior;
using namespace sofa::core::componentmodel::collision;

PipelineImpl::PipelineImpl()
{
}

PipelineImpl::~PipelineImpl()
{
}

void PipelineImpl::init()
{
	simulation::tree::GNode* root = dynamic_cast<simulation::tree::GNode*>(getContext());
	if(root == NULL) return;
	intersectionMethods.clear();
	root->getTreeObjects<Intersection>(&intersectionMethods);
	intersectionMethod = (intersectionMethods.empty() ? NULL : intersectionMethods[0]);
	broadPhaseDetections.clear();
	root->getTreeObjects<BroadPhaseDetection>(&broadPhaseDetections);
	broadPhaseDetection = (broadPhaseDetections.empty() ? NULL : broadPhaseDetections[0]);
	narrowPhaseDetections.clear();
	root->getTreeObjects<NarrowPhaseDetection>(&narrowPhaseDetections);
	narrowPhaseDetection = (narrowPhaseDetections.empty() ? NULL : narrowPhaseDetections[0]);
	contactManagers.clear();
	root->getTreeObjects<ContactManager>(&contactManagers);
	contactManager = (contactManagers.empty() ? NULL : contactManagers[0]);
	groupManagers.clear();
	root->getTreeObjects<CollisionGroupManager>(&groupManagers);
	groupManager = (groupManagers.empty() ? NULL : groupManagers[0]);

	if (intersectionMethod==NULL)
	{
		std::cerr << "WARNING(Pipeline): no intersectionMethod defined. Using DiscreteIntersection.\n";
		sofa::core::objectmodel::BaseObjectDescription discreteIntersectionDesc("Default Intersection","DiscreteIntersection");
		//intersectionMethod = dynamic_cast<Intersection*>(sofa::core::ObjectFactory::CreateObject(getContext(), &discreteIntersectionDesc));
		intersectionMethod = new sofa::component::collision::DiscreteIntersection;
	}
}

void PipelineImpl::reset()
{
    computeCollisionReset();
}

void PipelineImpl::computeCollisionReset()
{
	simulation::tree::GNode* root = dynamic_cast<simulation::tree::GNode*>(getContext());
	if(root == NULL) return;
	if (broadPhaseDetection!=NULL && broadPhaseDetection->getIntersectionMethod()!=intersectionMethod)
		broadPhaseDetection->setIntersectionMethod(intersectionMethod);
	if (narrowPhaseDetection!=NULL && narrowPhaseDetection->getIntersectionMethod()!=intersectionMethod)
		narrowPhaseDetection->setIntersectionMethod(intersectionMethod);
	if (contactManager!=NULL && contactManager->getIntersectionMethod()!=intersectionMethod)
		contactManager->setIntersectionMethod(intersectionMethod);
	doCollisionReset();
}

void PipelineImpl::computeCollisionDetection()
{
	simulation::tree::GNode* root = dynamic_cast<simulation::tree::GNode*>(getContext());
	if(root == NULL) return;
	std::vector<CollisionModel*> collisionModels;
	root->getTreeObjects<CollisionModel>(&collisionModels);
	doCollisionDetection(collisionModels);
}

void PipelineImpl::computeCollisionResponse()
{
	simulation::tree::GNode* root = dynamic_cast<simulation::tree::GNode*>(getContext());
	if(root == NULL) return;
	doCollisionResponse();
}

} // namespace tree

} // namespace simulation

} // namespace sofa
