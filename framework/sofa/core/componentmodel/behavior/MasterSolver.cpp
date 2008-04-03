#include <sofa/core/componentmodel/behavior/MasterSolver.h>
//#include <sofa/simulation/tree/MechanicalVisitor.h>
//#include <sofa/simulation/tree/CollisionVisitor.h>

#include <stdlib.h>
#include <math.h>

namespace sofa
{

namespace core
{

namespace componentmodel
{

namespace behavior
{

MasterSolver::MasterSolver()
{}

MasterSolver::~MasterSolver()
{}

#if 0

using namespace simulation::tree;

void MasterSolver::computeCollision()
{
    CollisionVisitor act;
    act.execute( getContext() );
}

void MasterSolver::integrate(double dt)
{
    MechanicalIntegrationVisitor act(dt);
    act.execute( getContext() );
}

#endif

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

