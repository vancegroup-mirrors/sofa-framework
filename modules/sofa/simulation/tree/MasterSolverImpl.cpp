#include "MasterSolverImpl.h"
#include <sofa/simulation/tree/MechanicalVisitor.h>
#include <sofa/simulation/tree/CollisionVisitor.h>

#include <stdlib.h>
#include <math.h>


namespace sofa
{

namespace simulation
{

namespace tree
{

MasterSolverImpl::MasterSolverImpl()
{}

MasterSolverImpl::~MasterSolverImpl()
{}

void MasterSolverImpl::computeCollision()
{
    CollisionVisitor act;
    act.execute( getContext() );
}

void MasterSolverImpl::integrate(double dt)
{
    MechanicalIntegrationVisitor act(dt);
    act.execute( getContext() );
}

} // namespace tree

} // namespace simulation

} // namespace sofa
