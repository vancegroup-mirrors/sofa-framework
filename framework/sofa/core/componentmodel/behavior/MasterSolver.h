#ifndef SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MASTERSOLVER_H
#define SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MASTERSOLVER_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

namespace componentmodel
{

namespace behavior
{

/**
 *  \brief Component responsible for main simulation algorithms, managing how
 *  and when collisions and integrations computations happen.
 *
 *  This class can optionally replace the default computation scheme of computing
 *  collisions then doing an integration step.
 *
 *  Note that it is in a preliminary stage, hence its fonctionnalities and API will
 *  certainly change soon.
 *
 */

class MasterSolver : public objectmodel::BaseObject
{
public:

    MasterSolver();

    virtual ~MasterSolver();

    /// Main computation method.
    ///
    /// Specify and execute all computations for computing a timestep, such
    /// as one or more collisions and integrations stages.
    virtual void step(double dt) = 0;

    /// @name Visitors
    /// These methods provides an abstract view of the mechanical system to animate.
    /// They are implemented by executing Visitors in the subtree of the scene-graph below this solver.
    /// @{

    /// Activate collision pipeline
    virtual void computeCollision() = 0;

    /// Activate OdeSolvers
    virtual void integrate(double dt) = 0;

    /// @}
};

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

#endif
