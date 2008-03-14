/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include "OdeSolverImpl.h"
#include <sofa/simulation/tree/MechanicalVisitor.h>
#include <sofa/simulation/tree/MechanicalMatrixVisitor.h>
#include <sofa/simulation/tree/MechanicalVPrintVisitor.h>
#include <sofa/simulation/tree/VelocityThresholdVisitor.h>


#include <stdlib.h>
#include <math.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

OdeSolverImpl::OdeSolverImpl()
: /*mat(NULL),*/ result(0)
{}

OdeSolverImpl::~OdeSolverImpl()
{}

double OdeSolverImpl::finish()
{
    return result;
}

OdeSolverImpl::VecId OdeSolverImpl::v_alloc(VecId::Type t)
{
    VecId v(t, vectors[t].alloc());
    MechanicalVAllocVisitor(v).execute( getContext() );
    return v;
}

void OdeSolverImpl::v_free(VecId v)
{
    if (vectors[v.type].free(v.index))
        MechanicalVFreeVisitor(v).execute( getContext() );
}

void OdeSolverImpl::v_clear(VecId v) ///< v=0
{
    MechanicalVOpVisitor(v).execute( getContext() );
}

void OdeSolverImpl::v_eq(VecId v, VecId a) ///< v=a
{
    MechanicalVOpVisitor(v,a).execute( getContext() );
}

void OdeSolverImpl::v_peq(VecId v, VecId a, double f) ///< v+=f*a
{
    MechanicalVOpVisitor(v,v,a,f).execute( getContext() );
}
void OdeSolverImpl::v_teq(VecId v, double f) ///< v*=f
{
    MechanicalVOpVisitor(v,VecId::null(),v,f).execute( getContext() );
}
void OdeSolverImpl::v_op(VecId v, VecId a, VecId b, double f) ///< v=a+b*f
{
    MechanicalVOpVisitor(v,a,b,f).execute( getContext() );
}

void OdeSolverImpl::v_dot(VecId a, VecId b) ///< a dot b ( get result using finish )
{
    result = 0;
    MechanicalVDotVisitor(a,b,&result).execute( getContext() );
}

void OdeSolverImpl::v_threshold(VecId a, double t) 
{
  VelocityThresholdVisitor(a,t).execute( getContext() );
}

void OdeSolverImpl::propagateDx(VecId dx)
{
    MechanicalPropagateDxVisitor(dx).execute( getContext() );
}

void OdeSolverImpl::projectResponse(VecId dx, double **W)
{
    MechanicalApplyConstraintsVisitor(dx, W).execute( getContext() );
}

void OdeSolverImpl::addMdx(VecId res, VecId dx, double factor)
{
    MechanicalAddMDxVisitor(res,dx,factor).execute( getContext() );
}

void OdeSolverImpl::integrateVelocity(VecId res, VecId x, VecId v, double dt)
{
    MechanicalVOpVisitor(res,x,v,dt).execute( getContext() );
}

void OdeSolverImpl::accFromF(VecId a, VecId f)
{
    MechanicalAccFromFVisitor(a,f).execute( getContext() );
}

void OdeSolverImpl::propagatePositionAndVelocity(double t, VecId x, VecId v)
{
    MechanicalPropagatePositionAndVelocityVisitor(t,x,v).execute( getContext() );
}

void OdeSolverImpl::computeForce(VecId result)
{
    MechanicalResetForceVisitor(result).execute( getContext() );
    finish();
    MechanicalComputeForceVisitor(result).execute( getContext() );
}

void OdeSolverImpl::computeDf(VecId df)
{
    MechanicalResetForceVisitor(df).execute( getContext() );
    finish();
    MechanicalComputeDfVisitor(df).execute( getContext() );
}

void OdeSolverImpl::computeDfV(VecId df)
{
    MechanicalResetForceVisitor(df).execute( getContext() );
    finish();
    MechanicalComputeDfVisitor(df,true).execute( getContext() );
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

void OdeSolverImpl::addSeparateGravity(double dt)
{
	MechanicalAddSeparateGravityVisitor(dt).execute( getContext() );
}

void OdeSolverImpl::computeContactForce(VecId result)
{
    MechanicalResetForceVisitor(result).execute( getContext() );
    finish();
    MechanicalComputeContactForceVisitor(result).execute( getContext() );
}

void OdeSolverImpl::computeContactDf(VecId df)
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

void OdeSolverImpl::print( VecId v, std::ostream& out )
{
    MechanicalVPrintVisitor(v,out).execute( getContext() );
}

void OdeSolverImpl::printWithElapsedTime( VecId v,  unsigned time, std::ostream& out )
{
    const double fact = 1000000.0 / (100*helper::system::thread::CTime::getTicksPerSec());
    MechanicalVPrintWithElapsedTimeVisitor(v,(int)((fact*time+0.5)*0.001), out).execute( getContext() );
}

// BaseMatrix & BaseVector Computations

void OdeSolverImpl::getMatrixDimension(unsigned int * const nbRow, unsigned int * const nbCol)
{
    MechanicalGetMatrixDimensionVisitor(nbRow, nbCol).execute( getContext() );
}

void OdeSolverImpl::addMBK_ToMatrix(defaulttype::BaseMatrix *A, double mFact, double bFact, double kFact, unsigned int offset)
{
    if (A != NULL)
    {
        MechanicalAddMBK_ToMatrixVisitor(A, mFact, bFact, kFact, offset).execute( getContext() );
    }
}

void OdeSolverImpl::addMBKdx_ToVector(defaulttype::BaseVector *V, VecId dx, double mFact, double bFact, double kFact, unsigned int offset)
{
	if (V != NULL)
		MechanicalAddMBKdx_ToVectorVisitor(V, dx, mFact, bFact, kFact, offset).execute( getContext() );
}

void OdeSolverImpl::multiVector2BaseVector(VecId src, defaulttype::BaseVector *dest, unsigned int offset)
{
    if (dest != NULL)
    {
        MechanicalMultiVector2BaseVectorVisitor(src, dest, offset).execute( getContext() );;
    }
}

void OdeSolverImpl::multiVectorPeqBaseVector(VecId dest, defaulttype::BaseVector *src, unsigned int offset)
{
	if (src != NULL)
	{
		MechanicalMultiVectorPeqBaseVectorVisitor(dest, src, offset).execute( getContext() );;
	}
}


} // namespace tree

} // namespace simulation

} // namespace sofa
