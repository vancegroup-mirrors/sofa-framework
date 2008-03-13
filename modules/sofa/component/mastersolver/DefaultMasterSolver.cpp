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
    if (this->f_printLog.getValue()) std::cout << "collision" << std::endl;
    computeCollision();
    // Then integrate the time step
    if (this->f_printLog.getValue()) std::cout << "integration" << std::endl;
    integrate(dt);
}

} // namespace mastersolver

} // namespace component

} // namespace sofa

