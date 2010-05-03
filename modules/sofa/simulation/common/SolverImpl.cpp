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
#include <sofa/simulation/common/SolverImpl.h>
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
    MechanicalVAvailVisitor(v).setTags(this->getTags()).execute( getContext() );
    MechanicalVAllocVisitor(v).setTags(this->getTags()).execute( getContext() );
    return v;
}

void SolverImpl::v_free(VecId v)
{
    //if (vectors[v.type].free(v.index))
        MechanicalVFreeVisitor(v).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::v_clear(VecId v) ///< v=0
{
    MechanicalVOpVisitor(v).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::v_eq(VecId v, VecId a) ///< v=a
{
    MechanicalVOpVisitor(v,a).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::v_peq(VecId v, VecId a, double f) ///< v+=f*a
{
    MechanicalVOpVisitor(v,v,a,f).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
}
void SolverImpl::v_teq(VecId v, double f) ///< v*=f
{
    MechanicalVOpVisitor(v,VecId::null(),v,f).setTags(this->getTags()).execute( getContext() );
}
void SolverImpl::v_op(VecId v, VecId a, VecId b, double f) ///< v=a+b*f
{
    MechanicalVOpVisitor(v,a,b,f).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
}

void SolverImpl::v_dot(VecId a, VecId b) ///< a dot b ( get result using finish )
{
    result = 0;
    MechanicalVDotVisitor(a,b,&result).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
}

void SolverImpl::v_threshold(VecId a, double t)
{
  VelocityThresholdVisitor(a,t).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::propagateDx(VecId dx)
{
    MechanicalPropagateDxVisitor(dx, false) //Don't ignore the masks
            .setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::propagateDxAndResetDf(VecId dx, VecId df)
{
    MechanicalPropagateDxAndResetForceVisitor(dx,df, false) //Don't ignore the masks
            .setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
    finish();
}

void SolverImpl::propagateX(VecId x)
{
    MechanicalPropagateXVisitor(x, false) //Don't ignore the masks
            .setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::propagateXAndResetF(VecId x, VecId f)
{
    MechanicalPropagateXAndResetForceVisitor(x,f, false) //Don't ignore the masks
            .setTags(this->getTags()).execute( getContext() );
    finish();
}

void SolverImpl::projectResponse(VecId dx, double **W)
{
    MechanicalApplyConstraintsVisitor(dx, W).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::addMdx(VecId res, VecId dx, double factor)
{
    MechanicalAddMDxVisitor(res,dx,factor).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::integrateVelocity(VecId res, VecId x, VecId v, double dt)
{
    MechanicalVOpVisitor(res,x,v,dt).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::accFromF(VecId a, VecId f)
{
    MechanicalAccFromFVisitor(a,f).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::computeForce(VecId result, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(result, false).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
	finish();
    }
    MechanicalComputeForceVisitor(result, accumulate).setTags(this->getTags()).execute( getContext() , true ); // enable prefetching
}

void SolverImpl::computeDf(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df).setTags(this->getTags()).execute( getContext() );
	finish();
    }
    MechanicalComputeDfVisitor(df, false, accumulate).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::computeDfV(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df).setTags(this->getTags()).execute( getContext() );
	finish();
    }
    MechanicalComputeDfVisitor(df, true, accumulate).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::addMBKdx(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df, true).setTags(this->getTags()).execute( getContext() );
	finish();
    }
    MechanicalAddMBKdxVisitor(df,m,b,k, false, accumulate).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
}

void SolverImpl::addMBKv(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	MechanicalResetForceVisitor(df, true).setTags(this->getTags()).execute( getContext() );
	finish();
    }
    MechanicalAddMBKdxVisitor(df,m,b,k, true, accumulate).setTags(this->getTags()).execute( getContext() );
}


void SolverImpl::addSeparateGravity(double dt, VecId result)
{
	MechanicalAddSeparateGravityVisitor(dt, result).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::computeContactForce(VecId result)
{
    MechanicalResetForceVisitor(result).setTags(this->getTags()).execute( getContext() );
    finish();
    MechanicalComputeContactForceVisitor(result).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::computeContactDf(VecId df)
{
    MechanicalResetForceVisitor(df).setTags(this->getTags()).execute( getContext() );
    finish();
    //MechanicalComputeDfVisitor(df).setTags(this->getTags()).execute( getContext() );
}


void SolverImpl::print( VecId v, std::ostream& out )
{
    MechanicalVPrintVisitor(v,out).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::printWithElapsedTime( VecId v,  unsigned time, std::ostream& out )
{
    const double fact = 1000000.0 / (100*helper::system::thread::CTime::getTicksPerSec());
    MechanicalVPrintWithElapsedTimeVisitor(v,(int)((fact*time+0.5)*0.001), out).setTags(this->getTags()).execute( getContext() );
}

// BaseMatrix & BaseVector Computations

void SolverImpl::getMatrixDimension(unsigned int * const nbRow, unsigned int * const nbCol)
{
    MechanicalGetMatrixDimensionVisitor(nbRow, nbCol).setTags(this->getTags()).execute( getContext() );
}

void SolverImpl::addMBK_ToMatrix(defaulttype::BaseMatrix *A, double mFact, double bFact, double kFact, unsigned int offset)
{
    if (A != NULL)
    {
        //std::cout << "MechanicalAddMBK_ToMatrixVisitor "<< mFact << " " << bFact << " " << kFact << " " << offset << std::endl;
        MechanicalAddMBK_ToMatrixVisitor(A, mFact, bFact, kFact, offset).setTags(this->getTags()).execute( getContext() );
    }
}
/*
void SolverImpl::addMBKdx_ToVector(defaulttype::BaseVector *V, VecId dx, double mFact, double bFact, double kFact, unsigned int offset)
{
	if (V != NULL)
		MechanicalAddMBKdx_ToVectorVisitor(V, dx, mFact, bFact, kFact, offset).setTags(this->getTags()).execute( getContext() );
}
*/
void SolverImpl::multiVector2BaseVector(VecId src, defaulttype::BaseVector *dest, unsigned int offset)
{
    if (dest != NULL)
    {
        MechanicalMultiVector2BaseVectorVisitor(src, dest, offset).setTags(this->getTags()).execute( getContext() );;
    }
}

void SolverImpl::multiVectorPeqBaseVector(VecId dest, defaulttype::BaseVector *src, unsigned int offset)
{
	if (src != NULL)
	{
		MechanicalMultiVectorPeqBaseVectorVisitor(dest, src, offset).setTags(this->getTags()).execute( getContext() );;
	}
}

} // namespace simulation

} // namespace sofa
