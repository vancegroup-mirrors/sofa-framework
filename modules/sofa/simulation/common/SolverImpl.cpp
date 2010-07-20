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
#include <sofa/core/behavior/LinearSolver.h>


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

void SolverImpl::prepareVisitor(Visitor* v)
{
    v->setTags(this->getTags());
}

void SolverImpl::prepareVisitor(MechanicalVisitor* v)
{
    if (v->writeNodeData())
        v->setNodeMap(this->getWriteNodeMap());
    else
        v->setNodeMap(this->getNodeMap());
    prepareVisitor((Visitor*)v);
}

SolverImpl::VecId SolverImpl::v_alloc(VecId::Type t)
{
    //VecId v(t, vectors[t].alloc());
    VecId v(t, VecId::V_FIRST_DYNAMIC_INDEX);
    executeVisitor( MechanicalVAvailVisitor(v) );
    executeVisitor( MechanicalVAllocVisitor(v) );
    return v;
}

void SolverImpl::v_free(VecId v)
{
    //if (vectors[v.type].free(v.index))
        executeVisitor( MechanicalVFreeVisitor(v) );
}

void SolverImpl::v_clear(VecId v) ///< v=0
{
    executeVisitor( MechanicalVOpVisitor(v) );
}

void SolverImpl::v_eq(VecId v, VecId a) ///< v=a
{
    executeVisitor( MechanicalVOpVisitor(v,a) );
}

void SolverImpl::v_peq(VecId v, VecId a, double f) ///< v+=f*a
{
    executeVisitor( MechanicalVOpVisitor(v,v,a,f), true ); // enable prefetching
}
void SolverImpl::v_teq(VecId v, double f) ///< v*=f
{
    executeVisitor( MechanicalVOpVisitor(v,VecId::null(),v,f) );
}
void SolverImpl::v_op(VecId v, VecId a, VecId b, double f) ///< v=a+b*f
{
    executeVisitor( MechanicalVOpVisitor(v,a,b,f), true ); // enable prefetching
}

void SolverImpl::v_dot(VecId a, VecId b) ///< a dot b ( get result using finish )
{
    result = 0;
    MechanicalVDotVisitor(a,b,&result).setNodeMap(this->getClearNodeMap()).setTags(this->getTags()).execute( getContext(), true ); // enable prefetching
}

void SolverImpl::v_threshold(VecId a, double t)
{
    executeVisitor( VelocityThresholdVisitor(a,t) );
}

void SolverImpl::propagateDx(VecId dx)
{
    executeVisitor( MechanicalPropagateDxVisitor(dx, false) //Don't ignore the masks
             );
}

void SolverImpl::propagateDxAndResetDf(VecId dx, VecId df)
{
    executeVisitor( MechanicalPropagateDxAndResetForceVisitor(dx,df, false) //Don't ignore the masks
            , true ); // enable prefetching
    finish();
}

void SolverImpl::propagateX(VecId x)
{
    executeVisitor( MechanicalPropagateXVisitor(x, false) //Don't ignore the masks
             );
}

void SolverImpl::propagateXAndResetF(VecId x, VecId f)
{
    executeVisitor( MechanicalPropagateXAndResetForceVisitor(x,f, false) //Don't ignore the masks
             );
    finish();
}

void SolverImpl::projectResponse(VecId dx, double **W)
{
    executeVisitor( MechanicalApplyConstraintsVisitor(dx, W) );
}

void SolverImpl::addMdx(VecId res, VecId dx, double factor)
{
    executeVisitor( MechanicalAddMDxVisitor(res,dx,factor) );
}

void SolverImpl::integrateVelocity(VecId res, VecId x, VecId v, double dt)
{
    executeVisitor( MechanicalVOpVisitor(res,x,v,dt) );
}

void SolverImpl::accFromF(VecId a, VecId f)
{
    executeVisitor( MechanicalAccFromFVisitor(a,f) );
}

void SolverImpl::computeForce(VecId result, bool clear, bool accumulate)
{
    if (clear)
    {
	executeVisitor( MechanicalResetForceVisitor(result, false), true ); // enable prefetching
	finish();
    }
    executeVisitor( MechanicalComputeForceVisitor(result, accumulate) , true ); // enable prefetching
}

void SolverImpl::computeDf(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	executeVisitor( MechanicalResetForceVisitor(df) );
	finish();
    }
    executeVisitor( MechanicalComputeDfVisitor(df, false, accumulate) );
}

void SolverImpl::computeDfV(VecId df, bool clear, bool accumulate)
{
    if (clear)
    {
	executeVisitor( MechanicalResetForceVisitor(df) );
	finish();
    }
    executeVisitor( MechanicalComputeDfVisitor(df, true, accumulate) );
}

void SolverImpl::addMBKdx(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	executeVisitor( MechanicalResetForceVisitor(df, true) );
	finish();
    }
    executeVisitor( MechanicalAddMBKdxVisitor(df,m,b,k, false, accumulate), true ); // enable prefetching
}

void SolverImpl::addMBKv(VecId df, double m, double b, double k, bool clear, bool accumulate)
{
    if (clear)
    {
	executeVisitor( MechanicalResetForceVisitor(df, true) );
	finish();
    }
    executeVisitor( MechanicalAddMBKdxVisitor(df,m,b,k, true, accumulate) );
}


void SolverImpl::addSeparateGravity(double dt, VecId result)
{
	executeVisitor( MechanicalAddSeparateGravityVisitor(dt, result) );
}

void SolverImpl::computeContactForce(VecId result)
{
    executeVisitor( MechanicalResetForceVisitor(result) );
    finish();
    executeVisitor( MechanicalComputeContactForceVisitor(result) );
}

void SolverImpl::computeContactDf(VecId df)
{
    executeVisitor( MechanicalResetForceVisitor(df) );
    finish();
    //executeVisitor( MechanicalComputeDfVisitor(df) );
}


void SolverImpl::print( VecId v, std::ostream& out )
{
    executeVisitor( MechanicalVPrintVisitor(v,out) );
}

void SolverImpl::printWithElapsedTime( VecId v,  unsigned time, std::ostream& out )
{
    const double fact = 1000000.0 / (100*helper::system::thread::CTime::getTicksPerSec());
    MechanicalVPrintWithElapsedTimeVisitor(v,(int)((fact*time+0.5)*0.001), out)/*.setNodeMap(this->getNodeMap())*/.setTags(this->getTags()).execute( getContext() );
}

// BaseMatrix & BaseVector Computations

void SolverImpl::getMatrixDimension(unsigned int * const nbRow, unsigned int * const nbCol, sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    executeVisitor( MechanicalGetMatrixDimensionVisitor(nbRow, nbCol, matrix) );
}

void SolverImpl::addMBK_ToMatrix(const sofa::core::behavior::MultiMatrixAccessor* matrix, double mFact, double bFact, double kFact)
{
    if (matrix != NULL)
    {
        //std::cout << "MechanicalAddMBK_ToMatrixVisitor "<< mFact << " " << bFact << " " << kFact << " " << offset << std::endl;
        executeVisitor( MechanicalAddMBK_ToMatrixVisitor(matrix, mFact, bFact, kFact) );
    }
}

void SolverImpl::multiVector2BaseVector(VecId src, defaulttype::BaseVector *dest, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    if (dest != NULL)
    {
        executeVisitor( MechanicalMultiVector2BaseVectorVisitor(src, dest, matrix) );
    }
}

void SolverImpl::multiVectorPeqBaseVector(VecId dest, defaulttype::BaseVector *src, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
	if (src != NULL)
	{
		executeVisitor( MechanicalMultiVectorPeqBaseVectorVisitor(dest, src, matrix) );
	}
}

} // namespace simulation

} // namespace sofa
