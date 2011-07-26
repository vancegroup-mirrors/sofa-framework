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
#ifndef SOFA_COMPONENT_MASTERSOLVER_FREEMOTIONMASTERSOLVER_H
#define SOFA_COMPONENT_MASTERSOLVER_FREEMOTIONMASTERSOLVER_H

#include <sofa/simulation/common/MasterSolverImpl.h>

#include <sofa/component/component.h>

namespace sofa
{
	namespace component
	{
		namespace constraintset
		{
			class LCPConstraintSolver;
		} // namespace constraintset
	} // namespace component
} // namespace sofa

namespace sofa
{

namespace component
{

namespace mastersolver
{

class SOFA_COMPONENT_MASTERSOLVER_API FreeMotionMasterSolver : public sofa::simulation::MasterSolverImpl
{
public:
	SOFA_CLASS(FreeMotionMasterSolver, sofa::simulation::MasterSolverImpl);

	FreeMotionMasterSolver();
	virtual ~FreeMotionMasterSolver();

	void step (const sofa::core::ExecParams* params /* PARAMS FIRST */, double dt);
	void init();

	virtual void parse ( sofa::core::objectmodel::BaseObjectDescription* arg );

	Data<bool> displayTime;

	Data<bool> m_solveVelocityConstraintFirst;

protected:
	sofa::core::behavior::ConstraintSolver *constraintSolver;
	component::constraintset::LCPConstraintSolver* defaultSolver;
};

} // namespace mastersolver

} // namespace component

} // namespace sofa

#endif
