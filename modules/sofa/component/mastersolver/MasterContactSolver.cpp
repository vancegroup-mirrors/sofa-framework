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
#include <sofa/simulation/common/SolveVisitor.h>

#include <sofa/core/ObjectFactory.h>

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
  {
  }

void MasterContactSolver::init()
{
  getContext()->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
}
  void MasterContactSolver::step(double dt)
  {
    simulation::Node *context =  (simulation::Node *)(this->getContext()); // access to current node
    CTime timer;
    CTime timerTotal;
    double time = 0.0;
    double timeTotal=0.0;
    double timeScale = 1000.0 / (double)CTime::getTicksPerSec();
    if ( displayTime.getValue() )
      {
        time = (double) timer.getTime();
        timeTotal = (double) timerTotal.getTime();
        //sout<<"********* Start Iteration : " << _numConstraints << " constraints *********" <<sendl;
      }

    // Update the BehaviorModels
    // Required to allow the RayPickInteractor interaction
	
    if( f_printLog.getValue())
      serr<<"updatePos called"<<sendl;

    simulation::BehaviorUpdatePositionVisitor updatePos(dt);
    context->execute(&updatePos);
	
    if( f_printLog.getValue())
      serr<<"updatePos performed - beginVisitor called"<<sendl;


    simulation::MechanicalBeginIntegrationVisitor beginVisitor(dt);
    context->execute(&beginVisitor);

    if( f_printLog.getValue())
      serr<<"beginVisitor performed - SolveVisitor for freeMotion is called"<<sendl;	
	
    // Free Motion
    simulation::SolveVisitor freeMotion(dt, true);
    context->execute(&freeMotion);
    simulation::MechanicalPropagateFreePositionVisitor().execute(context);
	
    if( f_printLog.getValue())
      serr<<" SolveVisitor for freeMotion performed"<<sendl;	


    if ( displayTime.getValue() )
      {
        sout << " >>>>> Begin display MasterContactSolver time" << sendl;
        sout<<" Free Motion " << ( (double) timer.getTime() - time)*timeScale <<" ms" <<sendl;

        time = (double) timer.getTime();
      }	 	

    // Collision detection and response creation
    computeCollision();

    if ( displayTime.getValue() )
      {
        sout<<" computeCollision " << ( (double) timer.getTime() - time)*timeScale <<" ms" <<sendl;
        time = (double) timer.getTime();
      }

    //SOLVE CONSTRAINT	
    if (constraintSolver) constraintSolver->solveConstraint(dt, core::VecId::freePosition());


    if ( displayTime.getValue() )
      {
        sout<<" contactCorrections " <<( (double) timer.getTime() - time)*timeScale <<" ms" <<sendl;
        sout << "<<<<<< End display MasterContactSolver time." << sendl;
      }


    simulation::MechanicalEndIntegrationVisitor endVisitor(dt);
    context->execute(&endVisitor);
  }


SOFA_DECL_CLASS(MasterContactSolver)

int MasterContactSolverClass = core::RegisterObject("Constraint solver")
.add< MasterContactSolver >()
;

} // namespace odesolver

} // namespace component

} // namespace sofa
