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
#include <sofa/component/odesolver/ComplianceArticulatedSystemSolver.h>
#include <sofa/simulation/tree/MechanicalVisitor.h>
#include <sofa/core/ObjectFactory.h>
#include <math.h>
#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using namespace core::componentmodel::behavior;

ComplianceArticulatedSystemSolver::ComplianceArticulatedSystemSolver()
: f_maxCGIter( initData(&f_maxCGIter,(unsigned)25,"iterations","Maximum number of iterations for the conjugated gradient algorithmIndices of the fixed points") )
, f_smallDenominatorThreshold( initData(&f_smallDenominatorThreshold,1e-5,"threshold","minimum value of the denominator in the conjugate Gradient solution") )
, firstCallToSolve( initData( &firstCallToSolve, true, "firstCallToSolve", "If true, the free movement is computed, if false, the constraint movement is computed"))
{
	dxfree = new NewMatVector();
	frame = 0;
}

void ComplianceArticulatedSystemSolver::init()
{
	context = dynamic_cast<simulation::tree::GNode *>(this->getContext()); // access to current node

	context->getTreeObject(ahc);

	articulations = dynamic_cast<component::MechanicalObject<Vec1dTypes>*>(context->getMechanicalState());	

	simulation::tree::GNode *c = *context->child.begin();
	Objects6D = dynamic_cast<component::MechanicalObject<RigidTypes>*>(c->getMechanicalState());

	dxfree->resize((*articulations->getX()).size());
	dxfree->element(0) = -0.01;
}

void ComplianceArticulatedSystemSolver::solve(double)
{
    OdeSolver* group = this;
	MultiVector force(group, VecId::force());
    MultiVector pos(group, VecId::position());
    MultiVector vel(group, VecId::velocity());
    MultiVector dx(group, VecId::dx());
	MultiVector posFree(group, VecId::freePosition());
	MultiVector velFree(group, VecId::freeVelocity());

 	if (!firstCallToSolve.getValue()) // f = contact force
	{
		// contact correction :
		simulation::tree::MechanicalPropagateAndAddDxVisitor(dx).execute(context);
	}
	else // f = mass * gravity
	{
		// Free motion :
		if(!ahc->chargedFromFile)
		{
			multiVectorPeqBaseVector(posFree, dxfree);
		}
		else
		{
			vector<sofa::component::container::ArticulatedHierarchyContainer::ArticulationCenter::Articulation*> art;
			context->getTreeObjects<sofa::component::container::ArticulatedHierarchyContainer::ArticulationCenter::Articulation>(&art);

			vector<sofa::component::container::ArticulatedHierarchyContainer::ArticulationCenter::Articulation*>::const_iterator a = art.begin();
			vector<sofa::component::container::ArticulatedHierarchyContainer::ArticulationCenter::Articulation*>::const_iterator aEnd = art.end();
			
			for (int i=0; a != aEnd; a++, i++)
			{
				if (i < 3){
					(*Objects6D->getXfree())[0].getCenter()[i] = (*a)->motion[frame];
				}
				else{
					(*articulations->getXfree())[i] = ((*a)->motion[frame]/180.0)*3.14;
					(*articulations->getX())[i] = ((*a)->motion[frame]/180.0)*3.14;
				}
			}
		}
		simulation::tree::MechanicalPropagateFreePositionVisitor().execute(context);
		if(ahc->chargedFromFile)
		{
			if (!(frame==ahc->numOfFrames-1))
			{
				frame++;
			}
		}
	}

	firstCallToSolve.setValue(!firstCallToSolve.getValue());
}

int ComplianceArticulatedSystemSolverClass = core::RegisterObject("Solver to test compliance computation for new articulated system objects")
    .add< ComplianceArticulatedSystemSolver >();

SOFA_DECL_CLASS(ComplianceArticulatedSystemSolver)


} // namespace odesolver

} // namespace component

} // namespace sofa

