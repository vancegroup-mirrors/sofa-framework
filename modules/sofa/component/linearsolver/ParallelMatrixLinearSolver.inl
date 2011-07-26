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
#ifndef SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_INL
#define SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_INL

#include <sofa/component/linearsolver/ParallelMatrixLinearSolver.h>
#ifdef SOFA_PARALLEL_UPDATE_LINEAR_SOLVER

#include <sofa/component/misc/ParallelizeBuildMatrixEvent.h>
#include <sofa/helper/AdvancedTimer.h>
#include <string.h>
#include <sofa/helper/system/thread/CTime.h>

//#define DEBUG_PARALLELMATRIX

namespace sofa {

namespace component {

namespace linearsolver {

using namespace helper::system::thread;
  
template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::ParallelMatrixLinearSolver()
: f_precompute( initData( &f_precompute, false, "precompute", "If true compute the system only at the initial step => linear deformation" ) )
, f_useWarping( initData( &f_useWarping, true, "useWarping", "use Warping around the solver" ) )
, f_useDerivative( initData( &f_useDerivative, false, "useDerivative", "use Derivative (A + epsilon X-1 = A-1 - epsilon A-1 X A-1 + O(epsilon2))" ) )
, f_useRotationFinder( initData( &f_useRotationFinder, (unsigned)0, "useRotationFinder", "Which rotation Finder to use" ) )
, f_useMultiThread( initData( &f_useMultiThread, true, "useMultiThread", "use MultiThraded version of the solver" ) )
, f_check_system( initData( &f_check_system, false, "check_system", "if true, check if the compliance matrix is correct (ie symmetric, diagonal != 0 and without nan)" ) )
, f_check_compliance( initData( &f_check_compliance, false, "check_compliance", "if true, check if the compliance matrix is correct (ie symmetric, diagonal != 0 and without nan)" ) )
{
    thread = NULL;
    systemRHVector = NULL;
    systemLHVector = NULL;
    Rcur = NULL;
    tmpVector1 = NULL;
    tmpVector2 = NULL;
    invertData[0] = NULL;
    invertData[1] = NULL;
    invertData[2] = NULL;
    matricesWork[0] = NULL;
    matricesWork[1] = NULL;
    matricesWork[2] = NULL;
    rotationWork[0] = NULL;
    rotationWork[1] = NULL;
    matrixAccessor[0] = NULL;
    matrixAccessor[1] = NULL;
    matrixAccessor[2] = NULL;
    tmpComputeCompliance[0] = NULL;
    tmpComputeCompliance[1] = NULL;
    solutionVecId = core::MultiVecDerivId::null();
    initialStep = true;
}

template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::~ParallelMatrixLinearSolver() {
    if (thread) {
      std::cout << "Wait for destoying thread" << std::endl;
      run = 0;
      while (!ready_thread) usleep(50);
      delete thread;
    }
    if (systemRHVector) delete systemRHVector;
    if (systemLHVector) delete systemLHVector;
    if (Rcur) delete Rcur;
    if (tmpVector1) delete tmpVector1;
    if (tmpVector2) delete tmpVector2;
    if (invertData[0]) delete invertData[0];
    if (invertData[1]) delete invertData[1];
    if (invertData[2]) delete invertData[2];
    if (matricesWork[0]) delete matricesWork[0];
    if (matricesWork[1]) delete matricesWork[1];
    if (matricesWork[2]) delete matricesWork[2];
    if (rotationWork[0]) delete rotationWork[0];
    if (rotationWork[1]) delete rotationWork[1];
    if (matrixAccessor[0]) delete matrixAccessor[0];
    if (matrixAccessor[1]) delete matrixAccessor[1];
    if (matrixAccessor[2]) delete matrixAccessor[2];
    if (tmpComputeCompliance[0]) delete tmpComputeCompliance[0];
    if (tmpComputeCompliance[1]) delete tmpComputeCompliance[1];
}

template<class TMatrix, class TVector>
void ParallelMatrixLinearSolver<TMatrix,TVector>::init() {
    sofa::core::objectmodel::BaseContext * c = this->getContext();
    c->get<sofa::component::misc::BaseRotationFinder >(&rotationFinders, sofa::core::objectmodel::BaseContext::Local);
    
    sout << "Found " << rotationFinders.size() << " Rotation finders" << sendl;
    for (unsigned i=0;i<rotationFinders.size();i++) {
      sout << i << " : " << rotationFinders[i]->getName() << sendl;
    }

    systemSize = 0;
    indexwork = 0;
    handeled = false;
    ready_thread = 1;
    nbstep_update = 1;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resetSystem() {
#ifdef DEBUG_PARALLELMATRIX
    printf(">resetSystem\n");
#endif
    if (!this->frozen && matricesWork[indexwork]) matricesWork[indexwork]->clear();
    if (systemRHVector) systemRHVector->clear();
    if (systemLHVector) systemLHVector->clear();
    solutionVecId = core::MultiVecDerivId::null();
#ifdef DEBUG_PARALLELMATRIX
    printf("<resetSystem\n");
#endif
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resizeSystem(int n) {
#ifdef DEBUG_PARALLELMATRIX
    printf(">resizeSystem\n");
#endif
    if (!systemRHVector) systemRHVector = createPersistentVector();
    if (!systemLHVector) systemLHVector = createPersistentVector();
    if (useRotation) {
	if (!Rcur) Rcur = createRotationMatrix();
	if (!tmpVector1) tmpVector1 = createPersistentVector();
    } 
    if (useDerivative) {
	if (!tmpVector1) tmpVector1 = createPersistentVector();
	if (!tmpVector2) tmpVector2 = createPersistentVector();
    }

    if (systemSize == n) return ;

    if (!this->frozen) {
      systemRHVector->resize(n);
      systemLHVector->resize(n);
      if (useRotation) {
	tmpVector1->resize(n);
      } 
      if (useDerivative) {
	tmpVector1->resize(n);
	tmpVector2->resize(n);
      }
    } 
    
    systemSize = n;
#ifdef DEBUG_PARALLELMATRIX
    printf("<resizeSystem\n");
#endif
}


template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::computeSystemMatrix(const core::MechanicalParams* mparams,int id) {
#ifdef DEBUG_PARALLELMATRIX
    printf(">computeSystemMatrix\n");
#endif
    if (!this->frozen) {
	  matricesWork[id]->clear();
	  matricesWork[id]->resize(systemSize,systemSize);
	  simulation::common::MechanicalOperations mops(mparams /* PARAMS FIRST */, this->getContext());	  
	  mops.addMBK_ToMatrix(matrixAccessor[id], mparams->mFactor(), mparams->bFactor(), mparams->kFactor());
	  matrixAccessor[id]->computeGlobalMatrix();
    }
    
    if (f_check_system.getValue()) {
      bool sym = true;
      bool diag = true;
      bool nan = false;
      for (unsigned i=0;i<matricesWork[id]->colSize();i++) {
	  for (unsigned j=0;j<matricesWork[id]->rowSize();j++) {
	      if (i==j) {
		if ((matricesWork[id]->element(j,i)>-0.00001) && (matricesWork[id]->element(j,i)<0.00001)) diag = false;
	      } else {
		double diff = matricesWork[id]->element(j,i) - matricesWork[id]->element(i,j);
		if ((diff<-0.00001) || (diff>0.00001)) sym = false;
	      }
	      if (isnan(matricesWork[id]->element(j,i))) {
		nan = true;
	      }
	  }
      }
      if (! sym) serr << "ERROR : THE SYSTEM MATRIX IS NOT SYMETRIX, CHECK THE METHOD addKToMatrix" << sendl;
      if (! diag) serr << "ERROR : THE SYSTEM MATRIX ZERO CONTAINS VALUE ON THE DIAGONAL" << sendl;
      if (nan) serr << "ERROR : THE SYSTEM MATRIX CONTAINS NAN VALUE" << sendl;
      if (sym && diag && !nan) serr << "THE SYSTEM MATRIX IS CORRECT" << sendl;
    }
#ifdef DEBUG_PARALLELMATRIX
    printf("<computeSystemMatrix\n");
#endif    
}

template<class Matrix, class Vector>
int ParallelMatrixLinearSolver<Matrix,Vector>::getDimension(const core::MechanicalParams* mparams,int id) {
#ifdef DEBUG_PARALLELMATRIX
    printf(">getDimension\n");
#endif    
    if (!matricesWork[id]) matricesWork[id] = createMatrix();
    if (!matrixAccessor[id]) matrixAccessor[id] = new DefaultMultiMatrixAccessor();  
    if (useRotation && !rotationWork[indexwork]) rotationWork[indexwork] = createRotationMatrix();
    
    simulation::common::MechanicalOperations mops(mparams /* PARAMS FIRST */, this->getContext());
    matrixAccessor[id]->setGlobalMatrix(matricesWork[id]); 
    matrixAccessor[id]->clear();
    mops.getMatrixDimension(matrixAccessor[id]);
    matrixAccessor[id]->setupMatrices();
#ifdef DEBUG_PARALLELMATRIX
    printf("<getDimension\n");
#endif       
    return matrixAccessor[id]->getGlobalDimension();
}


template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemMBKMatrix(const core::MechanicalParams* mparams) {	  
#ifdef DEBUG_PARALLELMATRIX
    printf(">setSystemMBKMatrix\n");
#endif
    if (initialStep ||  (! f_precompute.getValue())) {
	initialStep = false;

	useRotation = f_useWarping.getValue() && rotationFinders.size();
	useDerivative = f_useDerivative.getValue();
	indRotationFinder = f_useRotationFinder.getValue()<rotationFinders.size() ? f_useRotationFinder.getValue() : 0;
	
	if (f_useMultiThread.getValue() && (!f_precompute.getValue())) {	
	    if (ready_thread) {
		if (thread==NULL) {	
		    indexwork=0;
		    this->resizeSystem(this->getDimension(mparams,indexwork));
		    this->computeSystemMatrix(mparams,indexwork);
		    if (useRotation) rotationFinders[indRotationFinder]->getRotations(rotationWork[indexwork]);
		    this->invertSystem();
		    indexwork=1;	  
		    std::cout << "Creating thread ... " << std::endl;
		    thread = new boost::thread(Thread_invert(this));
		} else {
		  double time = ((double) CTime::getTime() - timer) / (double)CTime::getRefTicksPerSec();
		  sout << "Update preconditioner after " << nbstep_update << " steps in " << time << " ms" << sendl;
		}
		
		timer = CTime::getTime();
		
		this->resizeSystem(this->getDimension(mparams,indexwork));

		nbstep_update = 1;
		
		if (useDerivative) { // build the matrix in the current thread
		    computeSystemMatrix(mparams,indexwork);
		    handeled = false;
		    params = *mparams;
		} else {
		    sofa::component::misc::ParallelizeBuildMatrixEvent event;
		    this->getContext()->propagateEvent(core::ExecParams::defaultInstance(), &event);
		    handeled = event.isHandled();
		    if (! handeled) computeSystemMatrix(mparams,indexwork);
		    else params = *mparams;
		}
		
		if (indexwork) indexwork = 0;
		else indexwork = 1;
		
		ready_thread = 0; //unlock the second thread
		
		if (useRotation) rotationFinders[indRotationFinder]->getRotations(rotationWork[indexwork]);
		if (useDerivative) this->getDimension(mparams,2);		
	    } else nbstep_update++;
	} else {
	    resizeSystem(getDimension(mparams,indexwork));
	    this->computeSystemMatrix(mparams,indexwork);
	    this->invertSystem(); 
	    if (useRotation) rotationFinders[indRotationFinder]->getRotations(rotationWork[indexwork]);
	    if (useDerivative) this->getDimension(mparams,2);
	}
    }
#ifdef DEBUG_PARALLELMATRIX
    //std::cout << *matricesWork[indexwork] << std::endl;
    printf("<setSystemMBKMatrix\n");
#endif      

    this->updateSystemMatrix();
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemRHVector(core::MultiVecDerivId v) {
#ifdef DEBUG_PARALLELMATRIX
    printf(">setSystemRHVector\n");
#endif  
    this->executeVisitor( simulation::MechanicalMultiVectorToBaseVectorVisitor( core::ExecParams::defaultInstance(), v, systemRHVector, matrixAccessor[indexwork]) );
#ifdef DEBUG_PARALLELMATRIX
    printf("<setSystemRHVector\n");
#endif     
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemLHVector(core::MultiVecDerivId v) {
  #ifdef DEBUG_PARALLELMATRIX
    printf(">setSystemLHVector\n");
#endif  
    solutionVecId = v;
    //this->executeVisitor( simulation::MechanicalMultiVectorToBaseVectorVisitor( core::ExecParams::defaultInstance(), v, systemLHVector, matrixAccessor[indexwork]) );
#ifdef DEBUG_PARALLELMATRIX
    printf("<setSystemLHVector\n");
#endif      
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::invertSystem() {
#ifdef DEBUG_PARALLELMATRIX
    printf(">invertSystem\n");
#endif     
    this->invert(*matricesWork[indexwork]);
#ifdef DEBUG_PARALLELMATRIX
    printf("<invertSystem\n");
#endif       
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::updateSystemMatrix() {
#ifdef DEBUG_PARALLELMATRIX
    printf(">updateSystemMatrix\n");
#endif       
    if (useRotation) {
	rotationFinders[indRotationFinder]->getRotations(Rcur);
	rotationWork[indexwork]->opMulTM(Rcur,Rcur);
    } 
    
    if (useDerivative) {	
	this->computeSystemMatrix(&params,2); 
	internalData.subM(matricesWork[2],matricesWork[indexwork]);
    }
#ifdef DEBUG_PARALLELMATRIX
    printf("<updateSystemMatrix\n");
#endif       
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::solveSystem() {
#ifdef DEBUG_PARALLELMATRIX
    printf(">solveSystem\n");
#endif       
    
    if (useRotation && useDerivative) { 
	Rcur->opMulTV(systemLHVector,systemRHVector);
	
	this->solve(*matricesWork[indexwork], *tmpVector1, *systemLHVector);
	internalData.mulMV(matricesWork[2],tmpVector1,systemLHVector);
	this->solve(*matricesWork[indexwork], *tmpVector2, *systemLHVector);
	internalData.subV(tmpVector1,tmpVector2,tmpVector1);
		
	Rcur->opMulV(systemLHVector,tmpVector1);
    } else if (useRotation) {
	Rcur->opMulTV(systemLHVector,systemRHVector);
	this->solve(*matricesWork[indexwork], *tmpVector1, *systemLHVector);
	Rcur->opMulV(systemLHVector,tmpVector1);
    } else if (useDerivative) { 
	// see http://en.wikipedia.org/wiki/Invertible_matrix
	// (A + epsilon * X) -1  = A-1 - epsilon * A-1 * X * A -1  + O(epsilon²)
	
	// Solve(matricesWork[indexwork] * tmpVector1 = systemRHVector)  <=> tmpVector1 = matricesWork[indexwork]-1 * systemRHVector
	// systemLHVector = matricesWork[2] * tmpVector1
	// Solve(matricesWork[indexwork] * tmpVector2 = systemLHVector)  <=> tmpVector2 = matricesWork[indexwork]-1 * systemLHVector
	// systemLHVector = tmpVector1 - tmpVector2
	
	this->solve(*matricesWork[indexwork], *tmpVector1, *systemRHVector);
	internalData.mulMV(matricesWork[2],tmpVector1,systemLHVector);
	this->solve(*matricesWork[indexwork], *tmpVector2, *systemLHVector);
	internalData.subV(tmpVector1,tmpVector2,systemLHVector);
    } else {
	this->solve(*matricesWork[indexwork], *systemLHVector, *systemRHVector);
    }
    
    if (!solutionVecId.isNull()) {
	executeVisitor( simulation::MechanicalMultiVectorFromBaseVectorVisitor(core::ExecParams::defaultInstance(), solutionVecId, systemLHVector, matrixAccessor[indexwork]) );
    }
#ifdef DEBUG_PARALLELMATRIX
//     std::cout << *systemLHVector << std::endl;
    printf("<solveSystem\n");
#endif       
}

template<class Matrix, class Vector>
MatrixInvertData * ParallelMatrixLinearSolver<Matrix,Vector>::getMatrixInvertData(Matrix * m) {
    if (matricesWork[0] == m) {
      if (invertData[0] == NULL) invertData[0]=createInvertData();
      return invertData[0];
    }
    if (matricesWork[1] == m) {
      if (invertData[1] == NULL) invertData[1]=createInvertData();
      return invertData[1];
    }
    if (invertData[2] == NULL) invertData[2]=createInvertData();
    return invertData[2];
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
template<class Matrix, class Vector> template<class RMatrix, class JMatrix>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(Matrix & M, RMatrix& result, JMatrix& J, double fact) {
      const typename JMatrix::LineConstIterator jitend = J.end();
      // STEP 1 : put each line of matrix Jt in the right hand term of the system
      if (tmpComputeCompliance[0]==NULL) tmpComputeCompliance[0] = new Vector();
      if (tmpComputeCompliance[1]==NULL) tmpComputeCompliance[1] = new Vector();

      tmpComputeCompliance[1]->resize(J.colSize());
      for (typename JMatrix::LineConstIterator jit1 = J.begin(); jit1 != jitend; ++jit1) {
	  int row1 = jit1->first;
	  // clear the right hand term:
	  tmpComputeCompliance[0]->clear(); // currentGroup->systemMatrix->rowSize()
	  tmpComputeCompliance[0]->resize(J.colSize());
	  for (typename JMatrix::LElementConstIterator i1 = jit1->second.begin(), i1end = jit1->second.end(); i1 != i1end; ++i1) {
	    tmpComputeCompliance[0]->add(i1->first,i1->second);       
	  }

	  // STEP 2 : solve the system :
	  //solveSystem();
	  this->solve(M, *tmpComputeCompliance[1], *tmpComputeCompliance[0]);

	  // STEP 3 : project the result using matrix J
	  for (typename JMatrix::LineConstIterator jit2 = jit1; jit2 != jitend; ++jit2)
	  {
		  int row2 = jit2->first;
		  double acc = 0.0;
		  for (typename JMatrix::LElementConstIterator i2 = jit2->second.begin(), i2end = jit2->second.end(); i2 != i2end; ++i2)
		  {
			  int col2 = i2->first;
			  double val2 = i2->second;
			  acc += val2 * tmpComputeCompliance[1]->element(col2);
		  }
		  acc *= fact;
		  //sout << "W("<<row1<<","<<row2<<") += "<<acc<<" * "<<fact<<sendl;
		  result.add(row2,row1,acc);
		  if (row1!=row2) result.add(row1,row2,acc);
	  }	
      }
      return true; 
}

template<class Matrix, class Vector>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(Matrix * M, defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact) {
      if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J)) {
	return addJMInvJt(*M,*result,*j,fact);
      } else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J)) {
	return addJMInvJt(*M,*result,*j,fact);
      } 
      serr << "ERROR : Unknown matrix format in ParallelMatrixLinearSolver<Matrix,Vector>::addWarrpedJMInvJt" << sendl;
      return false;
}

template<class Matrix, class Vector>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact) {
#ifdef DEBUG_PARALLELMATRIX
    printf(">addJMInvJt\n");
#endif 	   
      const unsigned int Jrows = J->rowSize();
      const unsigned int Jcols = J->colSize();
      if (Jcols != matricesWork[indexwork]->rowSize()) {
	  serr << "ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt ERROR: incompatible J matrix size." << sendl;
	  return false;
      }
      
      bool res = false;
      if (Jrows) {
	if (useRotation) {
	    JR.resize(Jrows,Jcols);
	  
	    if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J)) {
	      Rcur->opMulJ(&JR,j);
	      res = addJMInvJt(matricesWork[indexwork],result,&JR,fact);
	    } else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J)) {
	      Rcur->opMulJ(&JR,j);
	      res = addJMInvJt(matricesWork[indexwork],result,&JR,fact);
	    } else {
	      serr << "ERROR : Unknown matrix format in ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt" << sendl;
	      res = false;
	    }
	} else {
	  res = addJMInvJt(matricesWork[indexwork],result,J,fact);
	}
      
	if (f_check_compliance.getValue() && res) {
	  bool sym = true;
	  bool diag = true;
	  bool nan = false;
	  for (unsigned i=0;i<result->colSize();i++) {
	      for (unsigned j=0;j<result->rowSize();j++) {
		  if (i==j) {
		    if ((result->element(j,i)>-1e-9) && (result->element(j,i)<1e-9)) diag = false;
		  } else {
		    double diff = result->element(j,i) - result->element(i,j);
		    if ((diff<-0.00001) || (diff>0.00001)) sym = false;
		  }
		  if (isnan(result->element(j,i))) {
		    nan = true;
		  }
	      }
	  }
	  if (! sym) serr << "ERROR : THE COMPLIANCE MATRIX IS NOT SYMETRIX, CHECK THE METHOD addKToMatrix" << sendl;
	  if (! diag) serr << "ERROR : THE COMPLIANCE MATRIX CONTAINS ZERO VALUE ON THE DIAGONAL" << sendl;
	  if (nan) serr << "ERROR : THE COMPLIANCE  MATRIX CONTAINS NAN VALUE" << sendl;
	  if (sym && diag && !nan) serr << "THE COMPLIANCE MATRIX IS CORRECT" << sendl;	
	}
      }
      
#ifdef DEBUG_PARALLELMATRIX
      printf("<addJMInvJt\n");      
#endif
      return res;

}

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
#endif
