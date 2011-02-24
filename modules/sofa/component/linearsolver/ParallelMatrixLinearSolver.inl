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

namespace sofa {

namespace component {

namespace linearsolver {

  
template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::ParallelMatrixLinearSolver()
: useWarping( initData( &useWarping, true, "useWarping", "use Warping around the solver" ) )
, useRotationFinder( initData( &useRotationFinder, (unsigned)0, "useRotationFinder", "Which rotation Finder to use" ) )
, useMultiThread( initData( &useMultiThread, true, "useMultiThread", "use MultiThraded version of the solver" ) )
, check_symetric( initData( &check_symetric, false, "check_symetric", "if true, check if the matrix is symetric" ) )
{
    thread = NULL;
    systemRHVector = NULL;
    systemLHVector = NULL;
    Rcur = NULL;
    tmpVectorRotation = NULL;
    invertData[0] = NULL;
    invertData[1] = NULL;
    invertData[2] = NULL;
    matricesWork[0] = NULL;
    matricesWork[1] = NULL;
    rotationWork[0] = NULL;
    rotationWork[1] = NULL;
    matrixAccessor[0] = NULL;
    matrixAccessor[1] = NULL;
    tmpComputeCompliance[0] = NULL;
    tmpComputeCompliance[1] = NULL;
    solutionVecId = core::MultiVecDerivId::null();
    useRotation = false;
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
    if (tmpVectorRotation) delete tmpVectorRotation;
    if (invertData[0]) delete invertData[0];
    if (invertData[1]) delete invertData[1];
    if (invertData[2]) delete invertData[2];
    if (matricesWork[0]) delete matricesWork[0];
    if (matricesWork[1]) delete matricesWork[1];
    if (rotationWork[0]) delete rotationWork[0];
    if (rotationWork[1]) delete rotationWork[1];
    if (matrixAccessor[0]) delete matrixAccessor[0];
    if (matrixAccessor[1]) delete matrixAccessor[1];
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
    useRotation = useWarping.getValue() && rotationFinders.size();
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resetSystem() {
    if (!this->frozen && matricesWork[indexwork]) matricesWork[indexwork]->clear();
    if (systemRHVector) systemRHVector->clear();
    if (systemLHVector) systemLHVector->clear();
    solutionVecId = core::MultiVecDerivId::null();
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resizeSystem(int n) {
    if (!systemRHVector) systemRHVector = createPersistentVector();
    if (!systemLHVector) systemLHVector = createPersistentVector();
    if (useRotation) {
	if (!Rcur) Rcur = createRotationMatrix();
	if (!tmpVectorRotation) tmpVectorRotation = createPersistentVector();
    }

    if (systemSize == n) return ;

    if (!this->frozen) {
      systemRHVector->resize(n);
      systemLHVector->resize(n);
      if (useRotation) tmpVectorRotation->resize(n);
    } 
    
    systemSize = n;
}


template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::computeSystemMatrix(const core::MechanicalParams* mparams,int id) {
   if (!this->frozen) {
	  matricesWork[id]->clear();
	  matricesWork[id]->resize(systemSize,systemSize);
	  simulation::common::MechanicalOperations mops(this->getContext(), mparams);	  
	  mops.addMBK_ToMatrix(matrixAccessor[id], mparams->mFactor(), mparams->bFactor(), mparams->kFactor());
	  matrixAccessor[id]->computeGlobalMatrix();
    }
    
    if (check_symetric.getValue()) {
      for (unsigned i=0;i<matricesWork[id]->colSize();i++) {
	  for (unsigned j=0;j<matricesWork[id]->rowSize();j++) {
	      double diff = matricesWork[id]->element(j,i) - matricesWork[id]->element(i,j);
	      if ((diff<-0.0000001) || (diff>0.0000001)) {
		serr << "ERROR : THE MATRIX IS NOT SYMETRIX, CHECK THE METHOD addKToMatrix" << sendl;
		return;
	      }
	  }
      }	
      serr << "THE MATRIX IS SYMETRIC" << sendl;
    }
}

template<class Matrix, class Vector>
int ParallelMatrixLinearSolver<Matrix,Vector>::getDimension(const core::MechanicalParams* mparams,int id) {
    if (!matricesWork[id]) matricesWork[id] = createMatrix();
    if (!matrixAccessor[id]) matrixAccessor[id] = new DefaultMultiMatrixAccessor();  
    if (useRotation && !rotationWork[indexwork]) rotationWork[indexwork] = createRotationMatrix();
    
    simulation::common::MechanicalOperations mops(this->getContext(), mparams);
    matrixAccessor[id]->setGlobalMatrix(matricesWork[id]); 
    matrixAccessor[id]->clear();
    mops.getMatrixDimension(matrixAccessor[id]);
    matrixAccessor[id]->setupMatrices();
    return matrixAccessor[id]->getGlobalDimension();
}


template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemMBKMatrix(const core::MechanicalParams* mparams) {	  
    useRotation = useWarping.getValue() && rotationFinders.size();
    indRotationFinder = useRotationFinder.getValue()<rotationFinders.size() ? useRotationFinder.getValue() : 0;
        
    resizeSystem(this->getDimension(mparams,indexwork));
    
    if (useRotation) rotationFinders[indRotationFinder]->getRotations(rotationWork[indexwork]);
        
    if (! useMultiThread.getValue()) {
	this->computeSystemMatrix(mparams,indexwork);
	this->invertSystem(); 
    } else if (ready_thread) {
	if (thread==NULL) {	
	  indexwork=0;
	  this->computeSystemMatrix(mparams,indexwork);
	  this->invertSystem();
	  indexwork=1;	  
	  getDimension(mparams,indexwork);// allocate and initialize data for second buffer  
	  thread = new boost::thread(Thread_invert(this));
	} else sout << "Update preconditioner after " << nbstep_update << " steps" << sendl;

	nbstep_update = 0;  	
	sofa::component::misc::ParallelizeBuildMatrixEvent event;
	this->getContext()->propagateEvent(&event);
	handeled = event.isHandled();
	
	if (! handeled) computeSystemMatrix(mparams,indexwork);
	else params = *mparams;

	if (indexwork) indexwork = 0;
	else indexwork = 1;
			
	ready_thread = 0; //unlock the second thread
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemRHVector(core::MultiVecDerivId v) {
    this->executeVisitor( simulation::MechanicalMultiVectorToBaseVectorVisitor( v, systemRHVector, matrixAccessor[indexwork]) );
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemLHVector(core::MultiVecDerivId v) {
    solutionVecId = v;
    this->executeVisitor( simulation::MechanicalMultiVectorToBaseVectorVisitor( v, systemLHVector, matrixAccessor[indexwork]) );
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::invertSystem() {
    this->invert(*matricesWork[indexwork]);
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::updateSystemMatrix() {
    nbstep_update++;
    
    if (useRotation) {
	rotationFinders[indRotationFinder]->getRotations(Rcur);
	rotationWork[indexwork]->opMulTM(Rcur,Rcur);
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::solveSystem() {
    if (useRotation) {
	    Rcur->opMulTV(systemLHVector,systemRHVector);
	    this->solve(*matricesWork[indexwork], *tmpVectorRotation, *systemLHVector);
	    Rcur->opMulV(systemLHVector,tmpVectorRotation);
    } else {
	    this->solve(*matricesWork[indexwork], *systemLHVector, *systemRHVector);
    }
    
    if (!solutionVecId.isNull()) {
	    executeVisitor( simulation::MechanicalMultiVectorFromBaseVectorVisitor(solutionVecId, systemLHVector, matrixAccessor[indexwork]) );
    }
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
bool ParallelMatrixLinearSolver<Matrix,Vector>::addWarrpedJMInvJt(Matrix * M, defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact) {
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
      const unsigned int Jrows = J->rowSize();
      const unsigned int Jcols = J->colSize();
      if (Jcols != matricesWork[indexwork]->rowSize()) {
	  serr << "ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt ERROR: incompatible J matrix size." << sendl;
	  return false;
      }

      if (!Jrows) return false;
  
      if (useRotation) {
	  JR.resize(Jrows,Jcols);     
	
	  if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J)) {
	    Rcur->opMulJ(&JR,j);
	  } else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J)) {
	    Rcur->opMulJ(&JR,j);
	  } else {
	    serr << "ERROR : Unknown matrix format in ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt" << sendl;
	    return false;
	  }
	  
	  return addWarrpedJMInvJt(matricesWork[indexwork],result,&JR,fact);
      } else return addWarrpedJMInvJt(matricesWork[indexwork],result,J,fact);
}

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
#endif
