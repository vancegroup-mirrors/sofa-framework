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
#ifndef SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_H
#define SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_H

#include <sofa/component/linearsolver/MatrixLinearSolver.h>

#ifdef WIN32
#define usleep(micro) Sleep(micro/1000)
#endif 

#ifndef SOFA_HAVE_BOOST //use previous declaration of matrixlinearsolver  

namespace sofa {

namespace component {

namespace linearsolver {

template<class Matrix, class Vector>
class SOFA_EXPORT_DYNAMIC_LIBRARY ParallelMatrixLinearSolver : public MatrixLinearSolver<Matrix,Vector> {
  public :
    SOFA_CLASS(SOFA_TEMPLATE2(ParallelMatrixLinearSolver,Matrix,Vector), SOFA_TEMPLATE2(MatrixLinearSolver,Matrix,Vector));  
    

    ParallelMatrixLinearSolver()
    : MatrixLinearSolver<Matrix,Vector>()
    {
    }
    
//     virtual bool addJMInvJt(defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact) {      
//       return false;
//     }

//     virtual bool addWarrpedJMInvJt(defaulttype::BaseMatrix* /*result*/, defaulttype::BaseMatrix* /*J*/, double /*fact*/) {
//       return false;
//     }
};

}

}

}

#else

#define SOFA_PARALLEL_UPDATE_LINEAR_SOLVER

#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/component/component.h>
#include <sofa/component/linearsolver/SparseMatrix.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/component/misc/BaseRotationFinder.h>
#include <sofa/component/linearsolver/RotationMatrix.h>
#include <sofa/component/linearsolver/DefaultMultiMatrixAccessor.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/core/behavior/BaseForceField.h>
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
	typedef SparseMatrix<Real> JRMatrixType;	
	
	void mulMV(const sofa::defaulttype::BaseMatrix * m,const TVector * v,TVector * res) {
		m->opMulV(res,v);
	}
	
	void subM(sofa::defaulttype::BaseMatrix * m1,const sofa::defaulttype::BaseMatrix * m2) {
		m2->opAddM(m1,-1.0);
	}	
	
	void subV(const TVector * v1,const TVector * v2,TVector * v3) {
		for (unsigned j=0;j<v1->size();j++) v3->set(j,v1->element(j)-v2->element(j));
	}	
};

template<class Matrix, class Vector>
class SOFA_EXPORT_DYNAMIC_LIBRARY ParallelMatrixLinearSolver : public BaseMatrixLinearSolver<Matrix, Vector> {
public:
	SOFA_CLASS(SOFA_TEMPLATE2(ParallelMatrixLinearSolver,Matrix,Vector), SOFA_TEMPLATE2(BaseMatrixLinearSolver,Matrix,Vector));
	
	typedef  std::list<int> ListIndex;
	typedef typename Matrix::Real Real;
	typedef typename ParallelMatrixLinearSolverInternalData<Vector>::TRotationMatrix TRotationMatrix;
	typedef typename ParallelMatrixLinearSolverInternalData<Vector>::JRMatrixType JRMatrixType;

	Data<bool> f_useWarping;
	Data<bool> f_useDerivative;
	Data<unsigned> f_useRotationFinder;
	Data<bool> f_useMultiThread;
	Data<bool> f_check_system;
	Data<bool> f_check_compliance;

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
	void setSystemMBKMatrix(const core::MechanicalParams* mparams);

	/// Set the linear system right-hand term vector, from the values contained in the (Mechanical/Physical)State objects
	void setSystemRHVector(core::MultiVecDerivId v);

	/// Set the initial estimate of the linear system left-hand term vector, from the values contained in the (Mechanical/Physical)State objects
	/// This vector will be replaced by the solution of the system once solveSystem is called
	void setSystemLHVector(core::MultiVecDerivId v);
	
	/// Get the linear system matrix, or NULL if this solver does not build it
	Matrix* getSystemMatrix() { return matricesWork[indexwork]; }

	/// Get the linear system right-hand term vector, or NULL if this solver does not build it
	Vector* getSystemRHVector() { return systemRHVector; }

	/// Get the linear system left-hand term vector, or NULL if this solver does not build it
	Vector* getSystemLHVector() { return systemLHVector; }

	/// Get the linear system matrix, or NULL if this solver does not build it
	defaulttype::BaseMatrix* getSystemBaseMatrix() { return matricesWork[indexwork]; }

	void updateSystemMatrix();

	/// Get the linear system right-hand term vector, or NULL if this solver does not build it
	defaulttype::BaseVector* getSystemRHBaseVector() { return systemRHVector; }

	/// Get the linear system left-hand term vector, or NULL if this solver does not build it
	defaulttype::BaseVector* getSystemLHBaseVector() { return systemLHVector; }

	/// Solve the system as constructed using the previous methods
	virtual void solveSystem();

	/// Invert the system, this method is optional because it's call when solveSystem() is called for the first time
	virtual void invertSystem();

	virtual std::string getTemplateName() const {
	    return templateName(this);
	}

	static std::string templateName(const ParallelMatrixLinearSolver<Matrix,Vector>* = NULL) {
	    return Matrix::Name();
	}

	virtual void invert(Matrix& ) {}
	
	void init();

	virtual void solve(Matrix& M, Vector& solution, Vector& rh) = 0;
	
	/// Default implementation for contacts
	/// @param result the variable where the result will be added
	/// @param J the matrix J to use
	/// @return false if the solver does not support this operation, of it the system matrix is not invertible
	virtual bool addJMInvJt(defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact);

	/// Warp the J matrix if necessary and call the  Default implementation
	/// @param result the variable where the result will be added
	/// @param J the matrix J to use
	/// @return false if the solver does not support this operation, of it the system matrix is not invertible
	virtual bool addJMInvJt(Matrix * M, defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact);
	
	/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
	template<class RMatrix, class JMatrix>
	bool addJMInvJt(Matrix & M, RMatrix& result, JMatrix& J, double fact);	
	
protected:

	/// newPartially solve the system
	virtual void partial_solve(Matrix& /*M*/, Vector& /*partial_solution*/, Vector& /*sparse_rh*/, ListIndex& /* indices_solution*/, ListIndex& /* indices input */) {}
	
	static Vector* createVector() {
	    return new Vector;
	}
	
	void prepareVisitor(Visitor* v) {
	    v->setTags(this->getTags());
	}

	void prepareVisitor(simulation::BaseMechanicalVisitor* v) {
            prepareVisitor((Visitor*)v);
	}
	
	template<class T>
	void executeVisitor(T v) {
	    prepareVisitor(&v);
	    v.execute( this->getContext() );
	}
	
	Vector* createPersistentVector() {
	    return new Vector;
	}

	Matrix* createMatrix() {
	    return new Matrix;
	}
	
	TRotationMatrix* createRotationMatrix() {
	    return new TRotationMatrix;
	}
	
	MatrixInvertData * getMatrixInvertData(Matrix * m);
	virtual MatrixInvertData * createInvertData() = 0;
	
	int getDimension(const core::MechanicalParams* mparams,int id);
	void computeSystemMatrix(const core::MechanicalParams* mparams,int id);	
	
	int indexwork;	
	bool useRotation;
	bool useDerivative;
	double timer;
	
	unsigned indRotationFinder;
	std::vector<sofa::component::misc::BaseRotationFinder *> rotationFinders;
	ParallelMatrixLinearSolverInternalData<Vector> internalData;
		
	int nbstep_update;
	Vector* systemRHVector;
	Vector* systemLHVector;
	TRotationMatrix * Rcur;
	Vector * tmpVector1;
	Vector * tmpVector2;
	
	int systemSize;
	Matrix * matricesWork[3]; 
	DefaultMultiMatrixAccessor * matrixAccessor[3];
	MatrixInvertData * invertData[3];
	TRotationMatrix * rotationWork[2];
	Vector * tmpComputeCompliance[2];
	
	JRMatrixType JR;
	core::MultiVecDerivId solutionVecId;
	core::MechanicalParams params;
	
	sofa::helper::system::atomic<int> handeled;
	sofa::helper::system::atomic<int> ready_thread;
	sofa::helper::system::atomic<int> run;
	

	class Thread_invert {
	    public :  
		Thread_invert(ParallelMatrixLinearSolver<Matrix,Vector> * s) {
			this->solver = s;
		}

		void operator()() {
			int indexwork = 1;
			solver->run=1;
			while (solver->run) {
				while (solver->ready_thread && solver->run) usleep(50);
				if (! solver->run) break;
				if (solver->handeled) solver->computeSystemMatrix(&solver->params,indexwork);
				if (! solver->run) break;
				solver->invert(*(solver->matricesWork[indexwork]));
				if (! solver->run) break;				
				solver->ready_thread = 1; //signal the main thread the invert is finish				
				if (indexwork) indexwork = 0;
				else indexwork = 1;				
			}
			solver->ready_thread = 1;
		} 

	    protected :
		ParallelMatrixLinearSolver<Matrix,Vector> * solver;
	}; 
	boost::thread * thread;
};

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif

#endif
