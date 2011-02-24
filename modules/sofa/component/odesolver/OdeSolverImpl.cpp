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
#include <sofa/component/odesolver/OdeSolverImpl.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/core/behavior/LinearSolver.h>

#include <sofa/defaulttype/Quat.h>


namespace sofa
{

namespace component
{

namespace odesolver
{
using core::VecId;
using linearsolver::FullVector;
using linearsolver::FullMatrix;

OdeSolverImpl::OdeSolverImpl()
{
}

void OdeSolverImpl::init()
{  
  OdeSolver::init();
  SolverImpl::init();
  reinit();
}

void OdeSolverImpl::propagatePositionAndVelocity(double t, VecId x, VecId v)
{
  simulation::MechanicalPropagatePositionAndVelocityVisitor(t,x,v).setTags(getTags()).execute( getContext() );
}

void OdeSolverImpl::computeAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, core::VecDerivId::force());
    propagatePositionAndVelocity(t, x, v);
    computeForce(f);

    if (f_printLog.getValue()) sout<<"OdeSolver::computeAcc, f = "<<f<<sendl;

    accFromF(a, f);
    projectResponse(a);
}


void OdeSolverImpl::computeContactAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, core::VecDerivId::force());
    propagatePositionAndVelocity(t, x, v);
    computeContactForce(f);
    if (f_printLog.getValue()) sout<<"OdeSolver::computeContactAcc, f = "<<f<<sendl;

    accFromF(a, f);
    projectResponse(a);
}

void OdeSolverImpl::solveConstraint(double dt, VecId id, core::behavior::BaseConstraintSet::ConstOrder order)
{
  simulation::Node *node=(simulation::Node*)getContext();
  for (simulation::Node::Sequence< core::behavior::ConstraintSolver >::iterator it=node->constraintSolver.begin(); it!=node->constraintSolver.end();++it)
    {
      (*it)->solveConstraint(dt, id, order);
    }
}

using sofa::core::behavior::LinearSolver;
using sofa::core::objectmodel::BaseContext;

void OdeSolverImpl::m_resetSystem()
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->resetSystem();
}

void OdeSolverImpl::m_setSystemMBKMatrix(double mFact, double bFact, double kFact)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemMBKMatrix(mFact, bFact, kFact);
}

void OdeSolverImpl::m_setSystemRHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemRHVector(v);
}

void OdeSolverImpl::m_setSystemLHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemLHVector(v);
}

void OdeSolverImpl::m_solveSystem()
{
  LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
//   std::cerr << getContext()<< " " << getContext()->getName() << " !!!!!! LinearSolver : " << this << "\n";
//   LinearSolver* s;
//   getContext()->get(s,BaseContext::SearchDown);
//   std::vector<core::objectmodel::BaseObject*> listObject;
//   getContext()->get<core::objectmodel::BaseObject>(&listObject, core::objectmodel::BaseContext::Local);
//   for (unsigned int i=0;i<listObject.size();++i) std::cerr << listObject[i]->getName() << "@" << listObject[i] << "\n";

//   std::vector<LinearSolver*> listLinear;
//   getContext()->get<LinearSolver>(&listLinear, core::objectmodel::BaseContext::Local);
//   for (unsigned int i=0;i<listLinear.size();++i) std::cerr << "LinearSolver " << listLinear[i]->getName() << "@" << listLinear[i] << "\n";
//   std::cerr << "S : " << s << "\n";
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->solveSystem();
}

void OdeSolverImpl::m_print( std::ostream& out )
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    defaulttype::BaseMatrix* m = s->getSystemBaseMatrix();
    if (!m) return;
    //out << *m;
    int ny = m->rowSize();
    int nx = m->colSize();
    out << "[";
    for (int y=0;y<ny;++y)
    {
        out << "[";
        for (int x=0;x<nx;x++)
            out << ' ' << m->element(x,y);
        out << "]";
    }
    out << "]";
}

} // namespace odesolver

} // namespace component

} // namespace sofa
