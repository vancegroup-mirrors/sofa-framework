/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/simulation/common/OdeSolverImpl.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/MechanicalMatrixVisitor.h>
#include <sofa/simulation/common/MechanicalVPrintVisitor.h>
#include <sofa/simulation/common/VelocityThresholdVisitor.h>
#include <sofa/core/componentmodel/behavior/LinearSolver.h>


#include <stdlib.h>
#include <math.h>

namespace sofa
{

namespace simulation
{


SolverImpl::SolverImpl()
: /*mat(NULL),*/ result(0)
{}

SolverImpl::~SolverImpl()
{}

double SolverImpl::finish()
{
    return result;
}

SolverImpl::VecId SolverImpl::v_alloc(VecId::Type t)
{
    //VecId v(t, vectors[t].alloc());
    VecId v(t, VecId::V_FIRST_DYNAMIC_INDEX);
    MechanicalVAvailVisitor(v).execute( getContext() );
    MechanicalVAllocVisitor(v).execute( getContext() );
    return v;
}

void SolverImpl::v_free(VecId v)
{
    //if (vectors[v.type].free(v.index))
        MechanicalVFreeVisitor(v).execute( getContext() );
}

void SolverImpl::v_clear(VecId v) ///< v=0
{
    MechanicalVOpVisitor(v).execute( getContext() );
}

void SolverImpl::v_eq(VecId v, VecId a) ///< v=a
{
    MechanicalVOpVisitor(v,a).execute( getContext() );
}

void SolverImpl::v_peq(VecId v, VecId a, double f) ///< v+=f*a
{
    MechanicalVOpVisitor(v,v,a,f).execute( getContext() );
}
void SolverImpl::v_teq(VecId v, double f) ///< v*=f
{
    MechanicalVOpVisitor(v,VecId::null(),v,f).execute( getContext() );
}
void SolverImpl::v_op(VecId v, VecId a, VecId b, double f) ///< v=a+b*f
{
    MechanicalVOpVisitor(v,a,b,f).execute( getContext() );
}

void SolverImpl::v_dot(VecId a, VecId b) ///< a dot b ( get result using finish )
{
    result = 0;
    MechanicalVDotVisitor(a,b,&result).execute( getContext() );
}

void SolverImpl::v_threshold(VecId a, double t) 
{
  VelocityThresholdVisitor(a,t).execute( getContext() );
}

void SolverImpl::propagateDx(VecId dx)
{
    MechanicalPropagateDxVisitor(dx).execute( getContext() );
}

void SolverImpl::propagateDxAndResetDf(VecId dx, VecId df)
{
    MechanicalPropagateDxAndResetForceVisitor(dx,df).execute( getContext() );
    finish();
}

void SolverImpl::projectResponse(VecId dx, double **W)
{
    MechanicalApplyConstraintsVisitor(dx, W).execute( getContext() );
}

void SolverImpl::addMdx(VecId res, VecId dx, double factor)
{
    MechanicalAddMDxVisitor(res,dx,factor).execute( getContext() );
}

void SolverImpl::integrateVelocity(VecId res, VecId x, VecId v, double dt)
{
    MechanicalVOpVisitor(res,x,v,dt).execute( getContext() );
}

void SolverImpl::accFromF(VecId a, VecId f)
{
    MechanicalAccFromFVisitor(a,f).execute( getContext() );
}

void OdeSolverImpl::propagatePositionAndVelocity(double t, VecId x, VecId v)
{
    MechanicalPropagatePositionAndVelocityVisitor(t,x,v).execute( getContext() );
}

void SolverImpl::computeForce(VecId result, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(result).execute( getContext() );
	finish();
    }
    MechanicalComputeForceVisitor(result, accumulate).execute( getContext() );
}

void SolverImpl::computeDf(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df).execute( getContext() );
	finish();
    }
    MechanicalComputeDfVisitor(df, false, accumulate).execute( getContext() );
}

void SolverImpl::computeDfV(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df).execute( getContext() );
	finish();
    }
    MechanicalComputeDfVisitor(df, true, accumulate).execute( getContext() );
}

void SolverImpl::addMBKdx(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df, true).execute( getContext() );
	finish();
    }
    MechanicalAddMBKdxVisitor(df,m,b,k, false, accumulate).execute( getContext() );
}

void SolverImpl::addMBKv(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df, true).execute( getContext() );
	finish();
    }
    MechanicalAddMBKdxVisitor(df,m,b,k, true, accumulate).execute( getContext() );
}

void OdeSolverImpl::computeAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, VecId::force());
    propagatePositionAndVelocity(t, x, v);
    computeForce(f);
    if( this->f_printLog.getValue()==true ){
        cerr<<"OdeSolver::computeAcc, f = "<<f<<endl;
    }

    accFromF(a, f);
    projectResponse(a);
}

void SolverImpl::addSeparateGravity(double dt)
{
	MechanicalAddSeparateGravityVisitor(dt).execute( getContext() );
}

void SolverImpl::computeContactForce(VecId result)
{
    MechanicalResetForceVisitor(result).execute( getContext() );
    finish();
    MechanicalComputeContactForceVisitor(result).execute( getContext() );
}

void SolverImpl::computeContactDf(VecId df)
{
    MechanicalResetForceVisitor(df).execute( getContext() );
    finish();
    //MechanicalComputeDfVisitor(df).execute( getContext() );
}

void OdeSolverImpl::computeContactAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, VecId::force());
    propagatePositionAndVelocity(t, x, v);
    computeContactForce(f);
    if( this->f_printLog.getValue()==true ){
        cerr<<"OdeSolver::computeContactAcc, f = "<<f<<endl;
    }

    accFromF(a, f);
    projectResponse(a);
}

void SolverImpl::print( VecId v, std::ostream& out )
{
    MechanicalVPrintVisitor(v,out).execute( getContext() );
}

void SolverImpl::printWithElapsedTime( VecId v,  unsigned time, std::ostream& out )
{
    const double fact = 1000000.0 / (100*helper::system::thread::CTime::getTicksPerSec());
    MechanicalVPrintWithElapsedTimeVisitor(v,(int)((fact*time+0.5)*0.001), out).execute( getContext() );
}

// BaseMatrix & BaseVector Computations

void SolverImpl::getMatrixDimension(unsigned int * const nbRow, unsigned int * const nbCol)
{
    MechanicalGetMatrixDimensionVisitor(nbRow, nbCol).execute( getContext() );
}

void SolverImpl::addMBK_ToMatrix(defaulttype::BaseMatrix *A, double mFact, double bFact, double kFact, unsigned int offset)
{
    if (A != NULL)
    {
        //std::cout << "MechanicalAddMBK_ToMatrixVisitor "<< mFact << " " << bFact << " " << kFact << " " << offset << std::endl;
        MechanicalAddMBK_ToMatrixVisitor(A, mFact, bFact, kFact, offset).execute( getContext() );
    }
}
/*
void SolverImpl::addMBKdx_ToVector(defaulttype::BaseVector *V, VecId dx, double mFact, double bFact, double kFact, unsigned int offset)
{
	if (V != NULL)
		MechanicalAddMBKdx_ToVectorVisitor(V, dx, mFact, bFact, kFact, offset).execute( getContext() );
}
*/
void SolverImpl::multiVector2BaseVector(VecId src, defaulttype::BaseVector *dest, unsigned int offset)
{
    if (dest != NULL)
    {
        MechanicalMultiVector2BaseVectorVisitor(src, dest, offset).execute( getContext() );;
    }
}

void SolverImpl::multiVectorPeqBaseVector(VecId dest, defaulttype::BaseVector *src, unsigned int offset)
{
	if (src != NULL)
	{
		MechanicalMultiVectorPeqBaseVectorVisitor(dest, src, offset).execute( getContext() );;
	}
}

using sofa::core::componentmodel::behavior::LinearSolver;
using sofa::core::objectmodel::BaseContext;

void OdeSolverImpl::m_resetSystem()
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
        return;
    }
    s->resetSystem();
}

void OdeSolverImpl::m_setSystemMBKMatrix(double mFact, double bFact, double kFact)
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
        return;
    }
    s->setSystemMBKMatrix(mFact, bFact, kFact);
}

void OdeSolverImpl::m_setSystemRHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
        return;
    }
    s->setSystemRHVector(v);
}

void OdeSolverImpl::m_setSystemLHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
        return;
    }
    s->setSystemLHVector(v);
}

void OdeSolverImpl::m_solveSystem()
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
        return;
    }
    s->solveSystem();
}

void OdeSolverImpl::m_print( std::ostream& out )
{
    LinearSolver* s = getContext()->get<LinearSolver>(BaseContext::SearchDown);
    if (!s)
    {
        std::cerr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<std::endl;
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

} // namespace simulation

} // namespace sofa
