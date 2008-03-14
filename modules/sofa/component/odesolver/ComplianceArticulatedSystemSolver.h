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
#ifndef SOFA_COMPONENT_ODESOLVER_COMPLIANCEARTICULATEDSYSTEMSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_COMPLIANCEARTICULATEDSYSTEMSOLVER_H

#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/tree/OdeSolverImpl.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/defaulttype/NewMatMatrix.h>
#include <sofa/defaulttype/NewMatVector.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/component/container/ArticulatedHierarchyContainer.h>


namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using core::objectmodel::Data;

/** 
* Solver to test compliance computation for new articulated system objects.
*/
class ComplianceArticulatedSystemSolver : public sofa::simulation::tree::OdeSolverImpl
{

public:
    Data<unsigned int> f_maxCGIter;
	Data<double> f_smallDenominatorThreshold;
	Data<bool> firstCallToSolve;
	simulation::tree::GNode *context;
	component::MechanicalObject<Vec1dTypes>* articulations;
	component::MechanicalObject<RigidTypes>* Objects6D;
	NewMatVector *dxfree;
	int frame;
	sofa::component::container::ArticulatedHierarchyContainer* ahc;

	void init();
	ComplianceArticulatedSystemSolver();

    void solve (double dt);
};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif
