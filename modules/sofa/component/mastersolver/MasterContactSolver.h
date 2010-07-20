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
#ifndef SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H

#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/simulation/common/MasterSolverImpl.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/behavior/ConstraintSolver.h>
#include <sofa/core/behavior/BaseConstraintCorrection.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/component/odesolver/OdeSolverImpl.h>
#include <sofa/component/constraintset/LCPConstraintSolver.h>
#include <sofa/helper/set.h>

namespace sofa
{

namespace component
{

namespace mastersolver
{

using namespace sofa::defaulttype;
using namespace sofa::component::linearsolver;
using namespace helper::system::thread;

class SOFA_COMPONENT_MASTERSOLVER_API MasterContactSolver : public sofa::simulation::MasterSolverImpl
{
public:
	SOFA_CLASS(MasterContactSolver, sofa::simulation::MasterSolverImpl);
        MasterContactSolver();
        void step (double dt);
        void init();

        virtual void parse ( sofa::core::objectmodel::BaseObjectDescription* arg );

        Data<bool> displayTime;
protected:
        sofa::core::behavior::ConstraintSolver *constraintSolver;
        constraintset::LCPConstraintSolver* defaultSolver;
};

} // namespace mastersolver

} // namespace component

} // namespace sofa

#endif
