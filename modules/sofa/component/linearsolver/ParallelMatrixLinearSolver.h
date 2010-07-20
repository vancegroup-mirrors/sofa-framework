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
#ifndef SOFA_HAVE_BOOST //use previous declaration of matrixlinearsolver

#include <sofa/component/linearsolver/MatrixLinearSolver.h>
#define ParallelMatrixLinearSolver MatrixLinearSolver
// namespace sofa {
// namespace component {
// namespace linearsolver {
// template<class Matrix, class Vector>
// class SOFA_EXPORT_DYNAMIC_LIBRARY ParallelMatrixLinearSolver : public SOFA_EXPORT_DYNAMIC_LIBRARY MatrixLinearSolver<Matrix,Vector> {
// public:
//     SOFA_CLASS(SOFA_TEMPLATE2(ParallelMatrixLinearSolver,Matrix,Vector),SOFA_TEMPLATE2(sofa::component::linearsolver::MatrixLinearSolver,Matrix,Vector));
// };
// }
// }
// }
#else 

#ifndef SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_H
#define SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_H
#include <sofa/simulation/common/SolverImpl.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/component/component.h>
#include <sofa/component/linearsolver/SparseMatrix.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/component/misc/BaseRotationFinder.h>
#include <sofa/component/linearsolver/RotationMatrix.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <sofa/helper/system/atomic.h>

namespace sofa {

namespace component {

namespace linearsolver {

template<class TVector>  
class ParallelMatrixLinearSolverInternalData {
  public :  
	typedef typename TVector::Real Real;
	typedef RotationMatrix<Real> TRotationMatrix;
};
 
template<class Matrix, class Vector>
class SOFA_EXPORT_DYNAMIC_LIBRARY ParallelMatrixLinearSolver : public sofa::core::behavior::LinearSolver, public sofa::simulation::SolverImpl
{
public:
	SOFA_CLASS2(SOFA_TEMPLATE2(ParallelMatrixLinearSolver,Matrix,Vector), sofa::core::behavior::LinearSolver, sofa::simulation::SolverImpl);
      
	typedef sofa::core::behavior::BaseMechanicalState::VecId VecId;
	typedef  std::list<int> ListIndex;
	typedef typename Matrix::Real Real;
	typedef typename ParallelMatrixLinearSolverInternalData<Vector>::TRotationMatrix TRotationMatrix;

	Data<bool> useWarping;
	Data<bool> useMultiThread;

	ParallelMatrixLinearSolver();
	virtual ~ParallelMatrixLinearSolver();

	/// Reset the current linear system.
	void resetSystem();

	/// Reset the current linear system.
	void resizeSystem(int n);

	/// Set the linear system matrix, combining the mechanical M,B,K matrices using the given coefficients
	///
	/// Note that this automatically resizes the linear system to the number of active degrees of freedoms
	///
	/// @todo Should we put this method in a specialized class for mechanical systems, or express it using more general terms (i.e. coefficients of the second order ODE to solve)
	void setSystemMBKMatrix(double mFact=0.0, double bFact=0.0, double kFact=0.0);

	/// Set the linear system right-hand term vector, from the values contained in the (Mechanical/Physical)State objects
	void setSystemRHVector(VecId v);

	/// Set the initial estimate of the linear system left-hand term vector, from the values contained in the (Mechanical/Physical)State objects
	/// This vector will be replaced by the solution of the system once solveSystem is called
	void setSystemLHVector(VecId v);

	/// Get the linear system matrix, or NULL if this solver does not build it
	Matrix* getSystemMatrix() { return matricesWork[indexwork]; }

	/// Get the linear system right-hand term vector, or NULL if this solver does not build it
	Vector* getSystemRHVector() { return systemRHVector; }

	/// Get the linear system left-hand term vector, or NULL if this solver does not build it
	Vector* getSystemLHVector() { return systemLHVector; }

	/// Get the linear system matrix, or NULL if this solver does not build it
	defaulttype::BaseMatrix* getSystemBaseMatrix() { return matricesWork[indexwork]; }

	/// Get the linear system right-hand term vector, or NULL if this solver does not build it
	defaulttype::BaseVector* getSystemRHBaseVector() { return systemRHVector; }

	/// Get the linear system left-hand term vector, or NULL if this solver does not build it
	defaulttype::BaseVector* getSystemLHBaseVector() { return systemLHVector; }

	/// Solve the system as constructed using the previous methods
	virtual void solveSystem();

	/// Invert the system, this method is optional because it's call when solveSystem() is called for the first time
	virtual void invertSystem() {
	    this->invert(*matricesWork[indexwork]);
	}

	virtual std::string getTemplateName() const
	{
	    return templateName(this);
	}

	static std::string templateName(const ParallelMatrixLinearSolver<Matrix,Vector>* = NULL)
	{
	    return Matrix::Name();
	}

	virtual void invert(Matrix& /*M*/) {}
	
	void init();

	virtual void solve(Matrix& M, Vector& solution, Vector& rh) = 0;

	/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
	///
	/// TODO : put this implementation in ParallelMatrixLinearSolver class - fix problems mith Scattered Matrix

	template<class RMatrix, class JMatrix>
	bool addJMInvJt(RMatrix& result, JMatrix& J, double fact);
	
	/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
	///
	/// @param result the variable where the result will be added
	/// @param J the matrix J to use
	/// @return false if the solver does not support this operation, of it the system matrix is not invertible
	bool addJMInvJt(defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact);

protected:

	/// newPartially solve the system
	virtual void partial_solve(Matrix& /*M*/, Vector& /*partial_solution*/, Vector& /*sparse_rh*/, ListIndex& /* indices_solution*/, ListIndex& /* indices input */) {}

	static Vector* createVector() {
	    return new Vector;
	}
	
	void computeSystemMatrix(double mFact, double bFact, double kFact);

	
	simulation::MultiNodeDataMap nodeMap;
	simulation::MultiNodeDataMap writeNodeMap;

	unsigned int systemSize;	    
	Matrix * matricesWork[2];
	TRotationMatrix * rotationWork[2];
	TRotationMatrix * Rcur;
	int indexwork;
	
	VecId solutionVecId;	
	Vector* systemRHVector;
	Vector* systemLHVector;
	
	bool useRotation;
	ParallelMatrixLinearSolverInternalData<Vector> internalData;
	std::vector<sofa::component::misc::BaseRotationFinder *> rotationFinders;	
	Vector tmpVectorRotation;
	
	bool first;
	boost::barrier * bar;
	sofa::helper::system::atomic<int> ready_thread;
	sofa::helper::system::atomic<int> run;
};

template<class Matrix, class Vector>
class Thread_invert {
public :  
	Thread_invert(boost::barrier * barg,
			  ParallelMatrixLinearSolver<Matrix,Vector> * solver,
			  sofa::helper::system::atomic<int> * ready_thread,
			  sofa::helper::system::atomic<int> * run,
			  Matrix ** matrices,
			  int index = 1
			  ) {
		this->bar = barg;
		this->solver = solver;
		this->ready_thread = ready_thread;
		this->run = run;
		this->matrices = matrices;
		this->index = index;
	}

	void operator()();

protected :
	boost::barrier * bar;
	ParallelMatrixLinearSolver<Matrix,Vector> * solver;
	sofa::helper::system::atomic<int> * ready_thread;
	sofa::helper::system::atomic<int> * run;
	Matrix ** matrices;
	int index;
}; 

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
#endif