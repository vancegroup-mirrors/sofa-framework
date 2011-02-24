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
#include <sofa/component/mastersolver/DefaultMasterSolver.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <math.h>
#include <iostream>




namespace sofa
{

namespace component
{

namespace mastersolver
{

int DefaultMasterSolverClass = core::RegisterObject("The simplest master solver, equivalent to the default behavior when no master solver is used.")
.add< DefaultMasterSolver >()
;

SOFA_DECL_CLASS(DefaultMasterSolver);

DefaultMasterSolver::DefaultMasterSolver(): firstCollision(initData(&firstCollision, true, "firstCollision", "First do the collision detection step, than simulate"))
{
}

DefaultMasterSolver::~DefaultMasterSolver()
{
}

void DefaultMasterSolver::step(double dt, const sofa::core::ExecParams* params)
{
  if (firstCollision.getValue())
  {
    // First we reset the constraints
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin constraints reset" << sendl;
    sofa::simulation::MechanicalResetConstraintVisitor(params).execute(this->getContext());
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end constraints reset" << sendl;
    // Then do collision detection and response creation
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin collision" << sendl;
    computeCollision(params);
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end collision" << sendl;
    // And finally integrate the time step
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin integration" << sendl;
    integrate(dt, params);
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end integration" << sendl;
  }
  else
  {
    // First integrate the time step
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin integration" << sendl;
    integrate(dt, params);
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end integration" << sendl;
    // Then we reset the constraints
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin constraints reset" << sendl;
    sofa::simulation::MechanicalResetConstraintVisitor(params).execute(this->getContext());
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end constraints reset" << sendl;
    // Finally do collision detection and response creation
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, begin collision" << sendl;
    computeCollision(params);
    if (this->f_printLog.getValue()) sout << "DefaultMasterSolver::step, end collision" << sendl;
  }
}

} // namespace mastersolver

} // namespace component

} // namespace sofa

