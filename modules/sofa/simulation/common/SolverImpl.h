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
#ifndef SOFA_SIMULATION_SOLVERIMPL_H
#define SOFA_SIMULATION_SOLVERIMPL_H

#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/core/behavior/MultiMatrixAccessor.h>
#include <sofa/simulation/common/common.h>

#ifdef SOFA_SMP
#include <sofa/core/behavior/ParallelMultivector.h>
using namespace sofa::defaulttype::SharedTypes;
#endif

namespace sofa
{

namespace simulation
{

class Visitor;
class MechanicalVisitor;

/**
 *  \brief Implementation of LinearSolver/OdeSolver/MasterSolver relying on component::System.
 *
 */
class SOFA_SIMULATION_COMMON_API SolverImpl : public virtual sofa::core::objectmodel::BaseObject
{
public:
    typedef sofa::core::behavior::BaseMechanicalState::VecId VecId;
    typedef std::map<core::objectmodel::BaseContext*, double> MultiNodeDataMap;

    SolverImpl();

    virtual ~SolverImpl();

    /// @name Visitors and MultiVectors
    /// These methods provides an abstract view of the mechanical system to animate.
    /// They are implemented by executing Visitors in the subtree of the scene-graph below this solver.
    /// @{

    /// @name Vector operations
    /// Most of these operations can be hidden by using the MultiVector class.
    /// @{

    /// Wait for the completion of previous operations and return the result of the last v_dot call.
    ///
    /// Note that currently all methods are blocking so finish simply return the result of the last v_dot call.
    virtual double finish();

    /// Allocate a temporary vector
    virtual VecId v_alloc(VecId::Type t);
    /// Free a previously allocated temporary vector
    virtual void v_free(VecId v);

    virtual void v_clear(VecId v); ///< v=0
    virtual void v_eq(VecId v, VecId a); ///< v=a
    virtual void v_peq(VecId v, VecId a, double f=1.0); ///< v+=f*a
#ifdef SOFA_SMP
    virtual void v_peq(VecId v, VecId a, Shared<double> &fSh, double f=1.0); ///< v+=f*a
    virtual void v_meq(VecId v, VecId a, Shared<double> &fSh); ///< v+=f*a
#endif
    virtual void v_teq(VecId v, double f); ///< v*=f
    virtual void v_op(VecId v, VecId a, VecId b, double f=1.0); ///< v=a+b*f
#ifdef SOFA_SMP
    virtual void v_op(VecId v, VecId a, VecId b, Shared<double> &f); ///< v=a+b*f
#endif

    virtual void v_dot(VecId a, VecId b); ///< a dot b ( get result using finish )
#ifdef SOFA_SMP
    virtual void v_dot(Shared<double> &result,VecId a, VecId b); ///< a dot b
#endif
    virtual void v_threshold(VecId a, double threshold); ///< nullify the values below the given threshold
    /// Propagate the given displacement through all mappings
    virtual void propagateDx(VecId dx);
    /// Propagate the given displacement through all mappings and reset the current force delta
    virtual void propagateDxAndResetDf(VecId dx, VecId df);
    /// Propagate the given position through all mappings
	virtual void propagateX(VecId x);
    /// Propagate the given position through all mappings and reset the current force delta
    virtual void propagateXAndResetF(VecId x, VecId f);
    /// Apply projective constraints to the given vector
    virtual void projectResponse(VecId dx, double **W=NULL);
    virtual void addMdx(VecId res, VecId dx=VecId(), double factor = 1.0); ///< res += factor M.dx
    virtual void integrateVelocity(VecId res, VecId x, VecId v, double dt); ///< res = x + v.dt
    virtual void accFromF(VecId a, VecId f); ///< a = M^-1 . f

    /// Compute the current force (given the latest propagated position and velocity)
    virtual void computeForce(VecId result, bool clear = true, bool accumulate = true);
    /// Compute the current force delta (given the latest propagated displacement)
    virtual void computeDf(VecId df, bool clear = true, bool accumulate = true);
    /// Compute the current force delta (given the latest propagated velocity)
    virtual void computeDfV(VecId df, bool clear = true, bool accumulate = true);
    /// accumulate $ df += (m M + b B + k K) dx $ (given the latest propagated displacement)
    virtual void addMBKdx(VecId df, double m, double b, double k, bool clear = true, bool accumulate = true);
    /// accumulate $ df += (m M + b B + k K) velocity $
    virtual void addMBKv(VecId df, double m, double b, double k, bool clear = true, bool accumulate = true);
    /// Add dt*Gravity to the velocity
    virtual void addSeparateGravity(double dt, VecId result=VecId::velocity());

    virtual void computeContactForce(VecId result);
    virtual void computeContactDf(VecId df);


    /// @}

    /// @name Matrix operations
    /// @{

    // BaseMatrix & BaseVector Computations
    virtual void getMatrixDimension(unsigned int * const, unsigned int * const, sofa::core::behavior::MultiMatrixAccessor* matrix = NULL);
    void getMatrixDimension(sofa::core::behavior::MultiMatrixAccessor* matrix)
    {
        getMatrixDimension(NULL, NULL, matrix);
    }
	virtual void addMBK_ToMatrix(const sofa::core::behavior::MultiMatrixAccessor* matrix, double mFact, double bFact, double kFact);
	virtual void multiVector2BaseVector(VecId src, defaulttype::BaseVector *dest, const sofa::core::behavior::MultiMatrixAccessor* matrix);
	virtual void multiVectorPeqBaseVector(VecId dest, defaulttype::BaseVector *src, const sofa::core::behavior::MultiMatrixAccessor* matrix);

    /// @}

    /// @name Debug operations
    /// @{

    /// Dump the content of the given vector.
    virtual void print( VecId v, std::ostream& out );
    virtual void printWithElapsedTime( VecId v,  unsigned time, std::ostream& out=std::cerr );

    /// @}

    /// @}

    /// @name Multi-Group operations
    /// @{

    virtual MultiNodeDataMap* getNodeMap()
    {
        return NULL;
    }

    virtual MultiNodeDataMap* getWriteNodeMap()
    {
        return NULL;
    }

    virtual MultiNodeDataMap* getClearNodeMap()
    {
        MultiNodeDataMap* m = getWriteNodeMap();
        if (m)
            for (MultiNodeDataMap::iterator it = m->begin(), end = m->end(); it != end; ++it)
                it->second = 0.0;
        return m;
    }
    
    /// @}

protected:

    virtual void prepareVisitor(Visitor* v);
    virtual void prepareVisitor(MechanicalVisitor* v);

    template<class T>
    void executeVisitor(T v)
    {
        prepareVisitor(&v);
        v.execute( this->getContext() );
    }

    template<class T>
    void executeVisitor(T* v)
    {
        prepareVisitor(v);
        v->execute( this->getContext() );
    }

    template<class T>
    void executeVisitor(T v, bool prefetch)
    {
        prepareVisitor(&v);
        v.execute( this->getContext(), prefetch );
    }

    template<class T>
    void executeVisitor(T* v, bool prefetch)
    {
        prepareVisitor(v);
        v->execute( this->getContext(), prefetch );
    }

    /// Result of latest v_dot operation
    double result;
};

} // namespace simulation

} // namespace sofa

#endif
