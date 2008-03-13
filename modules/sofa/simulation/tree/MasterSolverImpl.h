#ifndef SOFA_SIMULATION_TREE_MASTERSOLVERIMPL_H
#define SOFA_SIMULATION_TREE_MASTERSOLVERIMPL_H

#include <sofa/core/componentmodel/behavior/MasterSolver.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

/**
 *  \brief Implementation of MasterSolver relying on GNode.
 *
 *  Note that it is in a preliminary stage, hence its fonctionnalities and API will
 *  certainly change soon.
 *
 */

class MasterSolverImpl : public sofa::core::componentmodel::behavior::MasterSolver
{
public:

    MasterSolverImpl();

    virtual ~MasterSolverImpl();

    /// @name Visitors
    /// These methods provides an abstract view of the mechanical system to animate.
    /// They are implemented by executing Visitors in the subtree of the scene-graph below this solver.
    /// @{

    /// Activate collision pipeline
    virtual void computeCollision();

    /// Activate OdeSolvers
    virtual void integrate(double dt);

    /// @}
};

} // namespace tree

} // namespace simulation

} // namespace sofa

#endif
