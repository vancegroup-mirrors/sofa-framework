#ifndef SOFA_COMPONENT_MASTERSOLVER_DEFAULTMASTERSOLVER_H
#define SOFA_COMPONENT_MASTERSOLVER_DEFAULTMASTERSOLVER_H

#include <sofa/core/componentmodel/behavior/MasterSolver.h>
#include <sofa/simulation/tree/MasterSolverImpl.h>

namespace sofa
{

namespace component
{

namespace mastersolver
{

/** The simplest master solver, equivalent to the default behavior when no master solver is used.
*/
class DefaultMasterSolver : public sofa::simulation::tree::MasterSolverImpl
{
public:
    DefaultMasterSolver();
    void step (double dt);
};

} // namespace mastersolver

} // namespace component

} // namespace sofa

#endif
