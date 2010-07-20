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
#include <sofa/component/mastersolver/MasterContactSolver.h>

#include <sofa/simulation/common/AnimateVisitor.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/CollisionVisitor.h>
#include <sofa/simulation/common/SolveVisitor.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/VecId.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/system/thread/CTime.h>
#include <math.h>
#include <iostream>




namespace sofa
{

namespace component
{

namespace mastersolver
{

MasterContactSolver::MasterContactSolver()
: constraintSolver(NULL), defaultSolver(NULL)
{
}

void MasterContactSolver::parse ( sofa::core::objectmodel::BaseObjectDescription* arg )
{
    defaultSolver = new constraint::LCPConstraintSolver;
    defaultSolver->parse(arg);
}


void MasterContactSolver::init()
{
    getContext()->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
    if (constraintSolver == NULL && defaultSolver != NULL)
    {
        serr << "No ConstraintSolver found, using default LCPConstraintSolver" << sendl;
        this->getContext()->addObject(defaultSolver);
        constraintSolver = defaultSolver;
        defaultSolver = NULL;
    }
    else
    {
        delete defaultSolver;
        defaultSolver = NULL;
    }
}


void MasterContactSolver::step(double dt)
{
	using helper::system::thread::CTime;
	using sofa::helper::AdvancedTimer;

	simulation::Node *context = (simulation::Node *)(this->getContext()); // access to current node

	double time = 0.0;
	double timeTotal = 0.0;
	double timeScale = 1000.0 / (double)CTime::getTicksPerSec();

	if (displayTime.getValue())
	{
		time = (double) CTime::getTime();
		timeTotal = (double) CTime::getTime();
		//sout << "********* Start Iteration : " << _numConstraints << " constraints *********" << sendl;
	}

	context->execute< simulation::CollisionResetVisitor >();

	// Update the BehaviorModels
	// Required to allow the RayPickInteractor interaction

	if (f_printLog.getValue())
		serr << "updatePos called" << sendl;

	AdvancedTimer::stepBegin("UpdatePosition");
	simulation::BehaviorUpdatePositionVisitor updatePos(dt);
	context->execute(&updatePos);
	AdvancedTimer::stepEnd("UpdatePosition");

	if (f_printLog.getValue())
		serr << "updatePos performed - beginVisitor called" << sendl;

	simulation::MechanicalBeginIntegrationVisitor beginVisitor(dt);
	context->execute(&beginVisitor);

	if (f_printLog.getValue())
		serr << "beginVisitor performed - SolveVisitor for freeMotion is called" << sendl;	

	// Free Motion
	AdvancedTimer::stepBegin("FreeMotion");
	simulation::SolveVisitor freeMotion(dt, true);
	context->execute(&freeMotion);
	AdvancedTimer::stepBegin("PropagateFreePosition");
	simulation::MechanicalPropagateFreePositionVisitor().execute(context);
	AdvancedTimer::stepEnd("PropagateFreePosition");
	AdvancedTimer::stepEnd("FreeMotion");

	if (f_printLog.getValue())
		serr << " SolveVisitor for freeMotion performed" << sendl;

	if (displayTime.getValue())
	{
		sout << " >>>>> Begin display MasterContactSolver time" << sendl;
		sout <<" Free Motion " << ((double)CTime::getTime() - time) * timeScale << " ms" << sendl;

		time = (double)CTime::getTime();
	}	 	

	// Collision detection and response creation
	AdvancedTimer::stepBegin("Collision");
	computeCollision();
	AdvancedTimer::stepEnd  ("Collision");

	if (displayTime.getValue())
	{
		sout << " computeCollision " << ((double) CTime::getTime() - time) * timeScale << " ms" << sendl;
		time = (double)CTime::getTime();
	}

	// Restore force, intForce and extForce on the correct vectors (MState permanent VecIds)
	simulation::MechanicalResetForceVisitor resetForceVisitor(core::VecId::force());
	simulation::MechanicalResetForceVisitor resetIntForceVisitor(core::VecId::internalForce());
	simulation::MechanicalResetForceVisitor resetExtForceVisitor(core::VecId::externalForce());

	context->execute(&resetForceVisitor);
	context->execute(&resetIntForceVisitor);
	context->execute(&resetExtForceVisitor);

	// Solve constraints	
	if (constraintSolver)
	{
		AdvancedTimer::stepBegin("ConstraintSolver");
		constraintSolver->solveConstraint(dt, core::VecId::freePosition());
		AdvancedTimer::stepEnd("ConstraintSolver");
	}

	if ( displayTime.getValue() )
	{
		sout << " contactCorrections " << ((double)CTime::getTime() - time) * timeScale << " ms" <<sendl;
		sout << "<<<<<< End display MasterContactSolver time." << sendl;
	}

	simulation::MechanicalEndIntegrationVisitor endVisitor(dt);
	context->execute(&endVisitor);

	// Restore force, intForce and extForce on the correct vectors (MState permanent VecIds)
	context->execute(&resetForceVisitor);
	context->execute(&resetIntForceVisitor);
	context->execute(&resetExtForceVisitor);
}


SOFA_DECL_CLASS(MasterContactSolver)

int MasterContactSolverClass = core::RegisterObject("Constraint solver")
.add< MasterContactSolver >()
;

} // namespace odesolver

} // namespace component

} // namespace sofa
