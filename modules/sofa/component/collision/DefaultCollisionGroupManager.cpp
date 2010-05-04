/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/component/collision/DefaultCollisionGroupManager.h>
#include <sofa/component/collision/SolverMerger.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Simulation.h>

namespace sofa
{

namespace component
{

namespace collision
{

  using core::componentmodel::collision::Contact;


DefaultCollisionGroupManager::DefaultCollisionGroupManager()
{
}

DefaultCollisionGroupManager::~DefaultCollisionGroupManager()
{
}

simulation::Node* DefaultCollisionGroupManager::buildCollisionGroup() {
  return simulation::getSimulation()->newNode("CollisionGroup");
}

  
void DefaultCollisionGroupManager::createGroups(core::objectmodel::BaseContext* scene, const sofa::helper::vector<Contact*>& contacts)
{
	int groupIndex = 1;
	simulation::Node* node = dynamic_cast<simulation::Node*>(scene);
	if (node==NULL)
	{
		serr << "DefaultCollisionGroupManager only support graph-based scenes."<<sendl;
		return;
	}

	if (node && !node->getLogTime()) node=NULL; // Only use node for time logging
	simulation::Node::ctime_t t0 = 0;

	if (node) t0 = node->startTime();

	// Map storing group merging history
	std::map<simulation::Node*, simulation::Node*> mergedGroups;
	sofa::helper::vector<simulation::Node*> contactGroup;
	sofa::helper::vector<simulation::Node*> removedGroup;
	contactGroup.reserve(contacts.size());
	for(sofa::helper::vector<Contact*>::const_iterator cit = contacts.begin(); cit != contacts.end(); cit++)
	{
		Contact* contact = *cit;
                simulation::Node* group1 = getIntegrationNode(contact->getCollisionModels().first);
                simulation::Node* group2 = getIntegrationNode(contact->getCollisionModels().second);
		simulation::Node* group = NULL;
		if (group1==NULL || group2==NULL)
		{
		}
		else if (group1 == group2)
		{
			// same group, no new group necessary
			group = group1;
		}
		else if (simulation::Node* parent=findCommonParent(group1,group2))
		{
			// we can merge the groups
			// if solvers are compatible...
            bool mergeSolvers = (!group1->solver.empty() || !group2->solver.empty());
			SolverSet solver;
            if (mergeSolvers)
                solver = SolverMerger::merge(group1->solver[0], group2->solver[0]);
            //else std::cout << "New integration group below multi-group solver" << std::endl;
            if (!mergeSolvers || solver.odeSolver!=NULL)
			{
				bool group1IsColl = groupSet.find(group1)!=groupSet.end();
				bool group2IsColl = groupSet.find(group2)!=groupSet.end();
				if (!group1IsColl && !group2IsColl)
				{
					char groupName[32];
					snprintf(groupName,sizeof(groupName),"collision%d",groupIndex++);
					// create a new group
					group = buildCollisionGroup();
					group->setName(groupName);
					parent->addChild(group);

					core::objectmodel::Context *current_context = dynamic_cast< core::objectmodel::Context *>(parent->getContext());
					group->copyVisualContext( (*current_context));

					group->updateSimulationContext();
					group->moveChild((simulation::Node*)group1);
					group->moveChild((simulation::Node*)group2);
					groupSet.insert(group);
				}
				else if (group1IsColl)
				{
					group = group1;
					// merge group2 in group1
					if (!group2IsColl)
					{
						group->moveChild(group2);
					}
					else
					{
						// merge groups and remove group2
					    SolverSet solver2;
                        if (mergeSolvers)
                        {
                            solver2.odeSolver = group2->solver[0];
                            group2->removeObject(solver2.odeSolver);
                            if (!group2->linearSolver.empty())
                            {
                                solver2.linearSolver = group2->linearSolver[0];
                                group2->removeObject(solver2.linearSolver);
                            }
                            if (!group2->constraintSolver.empty())
                            {
                                solver2.constraintSolver = group2->constraintSolver[0];
                                group2->removeObject(solver2.constraintSolver);
                            }
                        }
                        while(!group2->object.empty())
                            group->moveObject(*group2->object.begin());
                        while(!group2->child.empty())
                            group->moveChild(*group2->child.begin());
                        parent->removeChild((simulation::Node*)group2);
                        groupSet.erase(group2);
                        mergedGroups[group2] = group;
                        if (solver2.odeSolver) delete solver2.odeSolver;
                        if (solver2.linearSolver) delete solver2.linearSolver;
                        if (solver2.constraintSolver) delete solver2.constraintSolver;
                        // BUGFIX(2007-06-23 Jeremie A): we can't remove group2 yet, to make sure the keys in mergedGroups are unique.
                        removedGroup.push_back(group2);
                        //delete group2;
					}
				}
				else
				{
					// group1 is not a collision group while group2 is
					group = group2;
					group->moveChild(group1);
				}
                if (!group->solver.empty())
				{
                    core::componentmodel::behavior::OdeSolver* solver2 = group->solver[0];
					group->removeObject(solver2);
					delete solver2;
				}
                if (!group->linearSolver.empty())
				{
					core::componentmodel::behavior::LinearSolver* solver2 = group->linearSolver[0];
					group->removeObject(solver2);
					delete solver2;
				}
                if (!group->constraintSolver.empty())
                {
                    core::componentmodel::behavior::ConstraintSolver* solver2 = group->constraintSolver[0];
                    group->removeObject(solver2);
                    delete solver2;
                }
                if (solver.linearSolver)
                    group->addObject(solver.odeSolver);
				if (solver.linearSolver)
				    group->addObject(solver.linearSolver);
                if (solver.constraintSolver)
				    group->addObject(solver.constraintSolver);
			}
		}
		contactGroup.push_back(group);
	}

	if (node) t0 = node->endTime(t0, "collision/groups", this);

	// now that the groups are final, attach contacts' response
	for(unsigned int i=0;i<contacts.size();i++)
	{
		Contact* contact = contacts[i];
		simulation::Node* group = contactGroup[i];
		while (group!=NULL && mergedGroups.find(group)!=mergedGroups.end())
			group = mergedGroups[group];
		if (group!=NULL)
			contact->createResponse(group);
		else
			contact->createResponse(scene);
	}

	if (node) t0 = node->endTime(t0, "collision/contacts", this);

	// delete removed groups
	for (sofa::helper::vector<simulation::Node*>::iterator it = removedGroup.begin(); it!=removedGroup.end(); ++it)
		delete *it;
	removedGroup.clear();

	// finally recreate group vector
	groups.clear();
	for (std::set<simulation::Node*>::iterator it = groupSet.begin(); it!=groupSet.end(); ++it)
		groups.push_back(*it);
	//if (!groups.empty())
	//	sout << groups.size()<<" collision groups created."<<sendl;
}

simulation::Node* DefaultCollisionGroupManager::getIntegrationNode(core::CollisionModel* model)
{
    simulation::Node* node = static_cast<simulation::Node*>(model->getContext());
    helper::vector< core::componentmodel::behavior::OdeSolver *> listSolver;
    node->get< core::componentmodel::behavior::OdeSolver >(&listSolver);
    
    if (listSolver.empty())
        return NULL;
    simulation::Node* solvernode = static_cast<simulation::Node*>(listSolver.back()->getContext());
    if (solvernode->linearSolver.empty())
        return solvernode; // no linearsolver
    core::componentmodel::behavior::LinearSolver * linearSolver = solvernode->linearSolver[0];
    if (!linearSolver->isMultiGroup())
    {
        //std::cout << "Linear solver " << linearSolver->getName() << " of CM " << model->getName() << " is not multi-group" << std::endl;
        return solvernode;
    }
    // This solver handles multiple groups, we have to find which group contains this collision model
    // First move up to the node of the initial mechanical object
    while (node->mechanicalMapping && node->mechanicalMapping->getMechFrom())
        node = static_cast<simulation::Node*>(node->mechanicalMapping->getMechFrom()->getContext());
    // Then check if it is one of the child nodes of the solver node
    for (simulation::Node::ChildIterator it = solvernode->child.begin(), itend = solvernode->child.end(); it != itend; ++it)
        if (*it == node)
        {
            //std::cout << "Group of CM " << model->getName() << " is " << (*it)->getName() << " child of " << solvernode->getName() << std::endl;
            return *it;
        }
    // Then check if it is a child of one of the child nodes of the solver node
    for (simulation::Node::ChildIterator it = solvernode->child.begin(), itend = solvernode->child.end(); it != itend; ++it)
        if (node->hasParent(*it))
            return *it;
    // Then check if it is a grand-childs of one of the child nodes of the solver node
    for (simulation::Node::ChildIterator it = solvernode->child.begin(), itend = solvernode->child.end(); it != itend; ++it)
        if (node->getContext()->hasAncestor(*it))
            return *it;
    // group not found, simply return the solver node
    return solvernode;
}

} // namespace collision

} // namespace component

} // namespace Sofa
