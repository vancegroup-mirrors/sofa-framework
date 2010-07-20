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
#ifndef SOFA_COMPONENT_ODESOLVER_ODESOLVERIMPL_H
#define SOFA_COMPONENT_ODESOLVER_ODESOLVERIMPL_H

#include <sofa/simulation/common/SolverImpl.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/core/behavior/BaseConstraintCorrection.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/simulation/common/MechanicalVisitor.h>


#include <stdlib.h>
#include <math.h>


namespace sofa
{

namespace component
{

namespace odesolver
{


class SOFA_COMPONENT_ODESOLVER_API OdeSolverImpl : public sofa::core::behavior::OdeSolver, public simulation::SolverImpl
{
public:
    typedef sofa::core::behavior::BaseMechanicalState::VecId VecId;
    typedef sofa::core::behavior::MultiVector<OdeSolverImpl> MultiVector;
    typedef sofa::core::behavior::MultiMatrix<OdeSolverImpl> MultiMatrix;
    typedef sofa::core::behavior::MechanicalMatrix MechanicalMatrix;
    typedef sofa::core::behavior::BaseLMConstraint::ConstOrder ConstOrder;


    OdeSolverImpl();
    virtual void init();
    /// Propagate the given state (time, position and velocity) through all mappings
    virtual void propagatePositionAndVelocity(double t, VecId x, VecId v);
    /// Compute the acceleration corresponding to the given state (time, position and velocity)
    virtual void computeAcc(double t, VecId a, VecId x, VecId v);
    virtual void computeContactAcc(double t, VecId a, VecId x, VecId v);

    virtual void solveConstraint(double /*dt*/, VecId);


    /// @name Matrix operations using LinearSolver components
    /// @{

    virtual void m_resetSystem();
    virtual void m_setSystemMBKMatrix(double mFact, double bFact, double kFact);
    virtual void m_setSystemRHVector(VecId v);
    virtual void m_setSystemLHVector(VecId v);
    virtual void m_solveSystem();
    virtual void m_print( std::ostream& out );

    /// @}
};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif
