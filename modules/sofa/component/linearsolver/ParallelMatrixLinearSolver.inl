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

#if 0 // TODO replace this line by the following one and fix this class
//#ifdef SOFA_HAVE_BOOST

#include <sofa/component/misc/ParallelizeBuildMatrixEvent.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa {

namespace component {

namespace linearsolver {

template<class Matrix, class Vector>
void Thread_invert<Matrix,Vector>::operator()() {	
	sharedData.run=1;
	while (sharedData.run) {
		while (sharedData.ready_thread && sharedData.run) usleep(50);
		
		if (! sharedData.run) break;

		if (sharedData.handeled) {
			matrixAccessor.setGlobalMatrix(sharedData.matricesWork[indexwork]);
			matrixAccessor.clear();
			solver->getMatrixDimension(&matrixAccessor);
			matrixAccessor.setupMatrices();
			unsigned systemSize = matrixAccessor.getGlobalDimension();
			sharedData.matricesWork[indexwork]->resize(systemSize,systemSize);
			//sharedData.matricesWork[indexwork]->clear();
			solver->addMBK_ToMatrix(&matrixAccessor, sharedData.mFact, sharedData.bFact, sharedData.kFact);
			matrixAccessor.computeGlobalMatrix();
		}		
		
		solver->invert(*(sharedData.matricesWork[indexwork]));
		
		if(indexwork) indexwork=0;
		else indexwork=1;
		
		sharedData.ready_thread = 1; //signal the main thread the invert is finish
	}	
	
	sharedData.ready_thread = 1;
}  
  
  
template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::ParallelMatrixLinearSolver()
: useWarping( initData( &useWarping, true, "useWarping", "use Warping around the solver" ) )
, useRotationFinder( initData( &useRotationFinder, (unsigned)0, "useRotationFinder", "Which rotation Finder to use" ) )
, useMultiThread( initData( &useMultiThread, true, "useMultiThread", "use MultiThraded version of the solver" ) )
, check_symetric( initData( &check_symetric, false, "check_symetric", "if true, check if the matrix is symetric" ) )
{
    thread = NULL;
    
    sharedData.matricesWork[0] = new Matrix();
    sharedData.rotationWork[0] = new TRotationMatrix();
    Rcur = new TRotationMatrix();
    systemLHVector = new Vector();
    systemRHVector = new Vector();
    
    sharedData.ready_thread = 1;
    indexwork = 0;
}

template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::~ParallelMatrixLinearSolver()
{
    if (thread) {
      std::cout << "Wait for destoying thread" << std::endl;
      sharedData.run = 0;
      while (!sharedData.ready_thread) sleep(100);
      delete thread;
    }
    
    if (systemRHVector) delete systemRHVector;
    if (systemLHVector) delete systemLHVector;
    if (Rcur) delete Rcur;
}

template<class TMatrix, class TVector>
void ParallelMatrixLinearSolver<TMatrix,TVector>::init() {
    sofa::core::objectmodel::BaseContext * c = this->getContext();

    c->get<sofa::component::misc::BaseRotationFinder >(&rotationFinders, sofa::core::objectmodel::BaseContext::Local);
    
    sout << "Found " << rotationFinders.size() << " Rotation finders" << sendl;
    for (unsigned i=0;i<rotationFinders.size();i++) {
      sout << i << " : " << rotationFinders[i]->getName() << sendl;
    }
    
    indexwork = 0;
    sharedData.ready_thread = 1;      
    sharedData.handeled = false;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resetSystem() {
    if (!this->frozen && sharedData.matricesWork[indexwork]) sharedData.matricesWork[indexwork]->clear();
    if (systemRHVector) systemRHVector->clear();
    if (systemLHVector) systemLHVector->clear();
    solutionVecId = VecId();
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resizeSystem(int n) {
    sharedData.matricesWork[indexwork]->resize(n,n);
    systemRHVector->resize(n);
    systemLHVector->resize(n);
    sharedData.systemSize = n;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::computeSystemMatrix() {
  if (!this->frozen) {
	matrixAccessor.setGlobalMatrix(sharedData.matricesWork[indexwork]);
	matrixAccessor.clear();
      
        this->getMatrixDimension(&matrixAccessor);
        matrixAccessor.setupMatrices();
        resizeSystem(matrixAccessor.getGlobalDimension());
        //sharedData.matricesWork[indexwork]->clear();
        this->addMBK_ToMatrix(&matrixAccessor, sharedData.mFact, sharedData.bFact, sharedData.kFact);
        matrixAccessor.computeGlobalMatrix();
  }
  if (useRotation) rotationFinders[indRotationFinder]->getRotations(sharedData.rotationWork[indexwork]);
  
  if (check_symetric.getValue()) {
    for (unsigned i=0;i<sharedData.matricesWork[indexwork]->colSize();i++) {
	for (unsigned j=0;j<sharedData.matricesWork[indexwork]->rowSize();j++) {
	    double diff = sharedData.matricesWork[indexwork]->element(j,i) - sharedData.matricesWork[indexwork]->element(i,j);
	    if ((diff<-0.0000001) || (diff>0.0000001)) {
	      std::cerr << "ERROR : THE MATRIX IS NOT SYMETRIX, CHECK THE METHOD addKToMatrix" << std::endl;
	      return;
	    }
	}
    }	
    std::cerr << "THE MATRIX IS SYMETRIC" << std::endl;
  }

}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemMBKMatrix(double mFact, double bFact, double kFact) {	  
	useRotation = useWarping.getValue() && rotationFinders.size();
	indRotationFinder = useRotationFinder.getValue()<rotationFinders.size() ? useRotationFinder.getValue() : 0;
	
	sharedData.mFact = mFact;
	sharedData.bFact = bFact;
	sharedData.kFact = kFact;

	if (! useMultiThread.getValue()) {
	    this->computeSystemMatrix();
	    this->invertSystem(); 
	} else if (sharedData.ready_thread) {
	    if (thread==NULL) {
		computeSystemMatrix();
		invertSystem(); //only this thread use metho invert.	    
		
		sharedData.matricesWork[1] = new Matrix();
		sharedData.rotationWork[1] = new TRotationMatrix();
		
		indexwork = 1;
	    }
	  
	    sofa::component::misc::ParallelizeBuildMatrixEvent event;
	    this->getContext()->propagateEvent(&event);
	    sharedData.handeled = event.isHandled();
	    
	    if (! sharedData.handeled) computeSystemMatrix();
	    else if (useRotation) rotationFinders[indRotationFinder]->getRotations(sharedData.rotationWork[indexwork]); //copy for the second thread at init				  
	    
	    sharedData.ready_thread = 0;//sharedData.bar->wait(); //launch the second thread if we use it      
	      
	    if (indexwork) indexwork=0;
	    else indexwork=1;
	    
	    matrixAccessor.setGlobalMatrix(sharedData.matricesWork[indexwork]);
	  
	    if (thread==NULL) {
		std :: cout << "Launching the invert thread...." << std::endl;// on matrix[1]
		thread = new Thread_invert<Matrix,Vector>(this,sharedData);
		boost::thread thrd(*thread);
	    } else {
		std::cout << "thread swap " << nbstep_update << " setSystemMBKMatrix in the preconditioner" << std::endl;
	    }
	    nbstep_update = 1;
	}
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemRHVector(VecId v) {
    this->multiVector2BaseVector(v, systemRHVector, &matrixAccessor);
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemLHVector(VecId v) {
    solutionVecId = v;
    this->multiVector2BaseVector(v, systemLHVector, &matrixAccessor);
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::updateSystemMatrix() {
	nbstep_update++;
	this->frozen = false;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::solveSystem() {
	if (useRotation && this->frozen) {
	    tmpVectorRotation.resize(sharedData.systemSize);  
	    rotationFinders[indRotationFinder]->getRotations(Rcur);
	    sharedData.rotationWork[indexwork]->opMulTM(Rcur,Rcur);
	}
	
	if (useRotation) {
		Rcur->opMulTV(systemLHVector,systemRHVector);
		this->solve(*sharedData.matricesWork[indexwork], tmpVectorRotation, *systemLHVector);
		Rcur->opMulV(systemLHVector,&tmpVectorRotation);
	} else {
		this->solve(*sharedData.matricesWork[indexwork], *systemLHVector, *systemRHVector);
	}
	
	if (!solutionVecId.isNull()) {
		v_clear(solutionVecId);
		multiVectorPeqBaseVector(solutionVecId, systemLHVector, &matrixAccessor);
	}
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
template<class Matrix, class Vector> template<class RMatrix, class JMatrix>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(RMatrix& result, JMatrix& J, double fact) {
  Rcur->opMulJ(&JR,&J);
  
  const unsigned int Jrows = JR.rowSize();
  const unsigned int Jcols = JR.colSize();
  if (Jcols != sharedData.matricesWork[indexwork]->rowSize()) {
      serr << "LULinearSolver::addJMInvJt ERROR: incompatible J matrix size." << sendl;
      return false;
  }

  if (!Jrows) return false;

  const typename SparseMatrix<Real>::LineConstIterator jitend = JR.end();
	  // STEP 1 : put each line of matrix Jt in the right hand term of the system
  for (typename SparseMatrix<Real>::LineConstIterator jit1 = JR.begin(); jit1 != jitend; ++jit1) {
      int row1 = jit1->first;
      // clear the right hand term:
      systemRHVector->clear(); // currentGroup->systemMatrix->rowSize()
      //double acc = 0.0;
      for (typename SparseMatrix<Real>::LElementConstIterator i1 = jit1->second.begin(), i1end = jit1->second.end(); i1 != i1end; ++i1) {
	systemRHVector->add(i1->first,i1->second);       
      }

      // STEP 2 : solve the system :
      solveSystem();


      // STEP 3 : project the result using matrix J
      for (typename SparseMatrix<Real>::LineConstIterator jit2 = jit1; jit2 != jitend; ++jit2)
      {
	      int row2 = jit2->first;
	      double acc = 0.0;
	      for (typename SparseMatrix<Real>::LElementConstIterator i2 = jit2->second.begin(), i2end = jit2->second.end(); i2 != i2end; ++i2)
	      {
		      int col2 = i2->first;
		      double val2 = i2->second;
		      acc += val2 * systemLHVector->element(col2);
	      }
	      acc *= fact;
	      //sout << "W("<<row1<<","<<row2<<") += "<<acc<<" * "<<fact<<sendl;
	      result.add(row2,row1,acc);
	      if (row1!=row2)
		  result.add(row1,row2,acc);
      }	
  }
  return true;
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
///
/// @param result the variable where the result will be added
/// @param J the matrix J to use
/// @return false if the solver does not support this operation, of it the system matrix is not invertible
template<class Matrix, class Vector>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(defaulttype::BaseMatrix* result, defaulttype::BaseMatrix* J, double fact)
{
    if (FullMatrix<double>* r = dynamic_cast<FullMatrix<double>*>(result))
    {
	if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
	else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
    }
    else if (FullMatrix<double>* r = dynamic_cast<FullMatrix<double>*>(result))
    {
	if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
	else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
    }
    else if (defaulttype::BaseMatrix* r = result)
    {
	if (SparseMatrix<double>* j = dynamic_cast<SparseMatrix<double>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
	else if (SparseMatrix<float>* j = dynamic_cast<SparseMatrix<float>*>(J))
	{
	    return addJMInvJt(*r,*j,fact);
	}
    }
    return false;
}

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
#endif
