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
#include <sofa/component/mastersolver/DefaultMasterSolver.h>
#include <sofa/core/ObjectFactory.h>
#include <math.h>
#include <iostream>

using std::cerr;
using std::endl;

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

DefaultMasterSolver::DefaultMasterSolver()
{
}

void DefaultMasterSolver::step(double dt)
{
    // First do collision detection and response creation
	if (this->f_printLog.getValue()) std::cout << "DefaultMasterSolver::step, begin collision" << std::endl;
    computeCollision();
	if (this->f_printLog.getValue()) std::cout << "DefaultMasterSolver::step, end collision" << std::endl;
    // Then integrate the time step
	if (this->f_printLog.getValue()) std::cout << "DefaultMasterSolver::step, begin integration" << std::endl;
    integrate(dt);
	if (this->f_printLog.getValue()) std::cout << "DefaultMasterSolver::step, end integration" << std::endl;
}

} // namespace mastersolver

} // namespace component

} // namespace sofa

