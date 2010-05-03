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
#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/core/componentmodel/behavior/LinearSolver.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/simulation/common/MechanicalVisitor.h>


#include <stdlib.h>
#include <math.h>

#ifdef SOFA_HAVE_EIGEN2
#include <Eigen/Core>
#include <Eigen/Sparse>
USING_PART_OF_NAMESPACE_EIGEN
#endif

namespace sofa
{

namespace component
{

namespace odesolver
{


class SOFA_COMPONENT_ODESOLVER_API OdeSolverImpl : public sofa::core::componentmodel::behavior::OdeSolver, public simulation::SolverImpl
{
public:
    typedef sofa::core::componentmodel::behavior::BaseMechanicalState::VecId VecId;
    typedef sofa::core::componentmodel::behavior::MultiVector<OdeSolverImpl> MultiVector;
    typedef sofa::core::componentmodel::behavior::MultiMatrix<OdeSolverImpl> MultiMatrix;
    typedef sofa::core::componentmodel::behavior::MechanicalMatrix MechanicalMatrix;
    typedef sofa::core::componentmodel::behavior::BaseLMConstraint::ConstOrder ConstOrder;


    OdeSolverImpl();
    virtual void init();
    /// Propagate the given state (time, position and velocity) through all mappings
    virtual void propagatePositionAndVelocity(double t, VecId x, VecId v);
    /// Compute the acceleration corresponding to the given state (time, position and velocity)
    virtual void computeAcc(double t, VecId a, VecId x, VecId v);
    virtual void computeContactAcc(double t, VecId a, VecId x, VecId v);

    /// @name Matrix operations using LinearSolver components
    /// @{

    virtual void m_resetSystem();
    virtual void m_setSystemMBKMatrix(double mFact, double bFact, double kFact);
    virtual void m_setSystemRHVector(VecId v);
    virtual void m_setSystemLHVector(VecId v);
    virtual void m_solveSystem();
    virtual void m_print( std::ostream& out );

    /// @}

    //Constraint solution using Eigen2
#ifdef SOFA_HAVE_EIGEN2

typedef Matrix<SReal, Eigen::Dynamic, Eigen::Dynamic> MatrixEigen;
typedef Matrix<SReal, Eigen::Dynamic, 1>              VectorEigen;
typedef Eigen::DynamicSparseMatrix<SReal,Eigen::RowMajor>    SparseMatrixEigen; 

    /// Explore the graph, looking for LMConstraints: each LMConstraint can tell if they need State Propagation in order to compute the right hand term of the system
    bool needPriorStatePropagation();
    /** Find all the LMConstraint present in the scene graph and solve a part of them
     * @param Id nature of the constraint to be solved
     **/
    void solveConstraint(bool priorStatePropagation, VecId Id);

 protected:
    /// Construct the Right hand term of the system
    void buildRightHandTerm( ConstOrder Order, const helper::vector< core::componentmodel::behavior::BaseLMConstraint* > &LMConstraints, VectorEigen &c);
    /** Apply the correction to the state corresponding
     * @param id nature of the constraint, and correction to apply
     * @param dof MechanicalState to correct
     * @param invM_Jtrans matrix M^-1.J^T to apply the correction from the independant dofs through the mapping
     * @param c correction vector
     * @param propageVelocityChange need to propagate the correction done to the velocity for the position
     **/
    void constraintStateCorrection(VecId &id, sofa::core::componentmodel::behavior::BaseMechanicalState* dof,
                                   const SparseMatrixEigen  &invM_Ltrans, const VectorEigen  &c, sofa::helper::set< unsigned int > &dofUsed);


 template <class T>
   class DofToMatrix
   {
   public:
   DofToMatrix(sofa::core::componentmodel::behavior::BaseMechanicalState *d,T m):dof(d), matrix(m)
     {}

     bool operator== (const sofa::core::componentmodel::behavior::BaseMechanicalState *other)
     {       
       if (other==dof) return true;
       else return false;
     }
     bool operator== (const DofToMatrix<T>& other)
     {
       if (&other != this)
         return dof == other.dof;
       else
         return true;
     }
     sofa::core::componentmodel::behavior::BaseMechanicalState *dof;
     T matrix;
   };
 
 std::vector< DofToMatrix< SparseMatrixEigen > > invMassMatrix;

#endif

};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif
