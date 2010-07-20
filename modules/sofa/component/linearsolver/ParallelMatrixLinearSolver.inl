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

#ifdef SOFA_COMPONENT_LINEARSOLVER_PARALLELMATRIXLINEARSOLVER_H

namespace sofa {

namespace component {

namespace linearsolver {

template<class Matrix, class Vector>
void Thread_invert<Matrix,Vector>::operator()() {	
	(*run)=1;
	while (*run) {
		solver->invert(*(matrices[index]));
		
		ready_thread[0] = 1; //signal the main thread the invert is finish
		
		if (*run) bar->wait();
		else break;
		
		if(index) index=0;
		else index=1;
	}
	
	bar->wait();
}  
  
  
template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::ParallelMatrixLinearSolver()
: useWarping( initData( &useWarping, true, "useWarping", "use Warping around the solver" ) )
, useMultiThread( initData( &useMultiThread, true, "useMultiThread", "use MultiThraded version of the solver" ) )
, check_symetric( initData( &check_symetric, false, "check_symetric", "if true, check if the matrix is symetric" ) )
{}

template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::~ParallelMatrixLinearSolver()
{
    run = 0;
    std::cout << "Wait for destoying thread" << std::endl;
    bar->wait();    
    
    if (systemRHVector) delete systemRHVector;
    if (systemLHVector) delete systemLHVector;
    
    if (matricesWork[0]) delete matricesWork[0];
    if (matricesWork[1]) delete matricesWork[1];
    if (rotationWork[0]) delete rotationWork[0];
    if (rotationWork[1]) delete rotationWork[1];
    if (Rcur) delete Rcur;
    delete bar;
}


template<class TMatrix, class TVector>
void ParallelMatrixLinearSolver<TMatrix,TVector>::init() {
    sofa::core::objectmodel::BaseContext * c = this->getContext();

    c->get<sofa::component::misc::BaseRotationFinder >(&rotationFinders, sofa::core::objectmodel::BaseContext::Local);
    
    sout << "Found " << rotationFinders.size() << " Rotation finders" << sendl;
    
    first = true;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resetSystem() {
    if (!frozen && matricesWork[indexwork]) matricesWork[indexwork]->clear();
    if (systemRHVector) systemRHVector->clear();
    if (systemLHVector) systemLHVector->clear();
    solutionVecId = VecId();
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resizeSystem(int n) {
    matricesWork[indexwork]->resize(n,n);
    systemRHVector->resize(n);
    systemLHVector->resize(n);
    systemSize = n;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::computeSystemMatrix(double mFact, double bFact, double kFact) {
  if (!this->frozen) {
      unsigned int nbRow=0, nbCol=0;
      this->getMatrixDimension(&nbRow,&nbCol);
      resizeSystem(nbRow);
      matricesWork[indexwork]->clear();
      unsigned int offset = 0;
      this->addMBK_ToMatrix(matricesWork[indexwork], mFact, bFact, kFact, offset);
  }
  for (unsigned i=0;i<useRotation && rotationFinders.size();i++) rotationFinders[i]->getRotations(rotationWork[indexwork]);
  
  if (check_symetric.getValue()) {
    for (unsigned i=0;i<matricesWork[indexwork]->colSize();i++) {
	for (unsigned j=0;j<matricesWork[indexwork]->rowSize();j++) {
	    double diff = matricesWork[indexwork]->element(j,i) - matricesWork[indexwork]->element(i,j);
	    if ((diff<-0.0000001) || (diff>0.0000001)) {
	      printf("ERROR : THE MATRIX IS NOT SYMETRIX, CHECK THE METHOD addKToMatrix\n");
	      return;
	    }
	}
    }	
    printf("THE MATRIX IS SYMETRIC\n");
  }

}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemMBKMatrix(double mFact, double bFact, double kFact) {
	useRotation = useWarping.getValue() && rotationFinders.size();
    
	if (first) {
	    first = false;
	    matricesWork[0] = new Matrix();
	    matricesWork[1] = new Matrix();
	    rotationWork[0] = new TRotationMatrix();
	    rotationWork[1] = new TRotationMatrix();
	    Rcur = new TRotationMatrix();
	    systemLHVector = new Vector();
	    systemRHVector = new Vector();

	    
	    //////////////////////////////////////////////////////////////
	    //CompressedRowSparseMatrix doesn't store data until the first "non zero resize" plus clear 
	    //When the bug will be fix thooses linge should be remove
	    unsigned int nbRow=0, nbCol=0;
	    this->getMatrixDimension(&nbRow,&nbCol);
	    matricesWork[0]->resize(nbRow,nbRow);matricesWork[0]->clear();	    
	    matricesWork[1]->resize(nbRow,nbRow);matricesWork[1]->clear();    
	    //////////////////////////////////////////////////////////////
	    
	    indexwork = 0;
	    this->computeSystemMatrix(mFact,bFact,kFact);
	    this->invertSystem(); //only this thread use metho invert.
	    
	    indexwork = 1;
	    this->computeSystemMatrix(mFact,bFact,kFact);
	    
	    ready_thread = 0;
	    bar = new boost::barrier(2);
	    std :: cout << "Launching the invert thread...." << std::endl;// on matrix[1]
	    Thread_invert<Matrix,Vector> thi(bar,this,&ready_thread,&run,matricesWork,indexwork);
	    boost::thread thrd(thi);
	    
	    indexwork = 0;
	    nbstep_update = 0;
	    
	} else if (! useMultiThread.getValue()) {
	    this->computeSystemMatrix(mFact,bFact,kFact);		    
	    this->invertSystem();
	} else if (ready_thread) {
	    ready_thread = 0;
	    	    
	    this->computeSystemMatrix(mFact,bFact,kFact);
	    	    
	    bar->wait();
	    
	    std::cout << "thread swap " << nbstep_update << " needed" << std::endl;
	    nbstep_update = 0;
	    
	    if (indexwork) indexwork=0;
	    else indexwork=1;	    
	} else nbstep_update++;
	
	if (useRotation) tmpVectorRotation.resize(systemSize);  
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemRHVector(VecId v) {
    unsigned int offset = 0;
    this->multiVector2BaseVector(v, systemRHVector, offset);
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemLHVector(VecId v) {
    solutionVecId = v;
    unsigned int offset = 0;
    this->multiVector2BaseVector(v, systemLHVector, offset);
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::solveSystem() {
	if (useRotation && frozen) {
	    for (unsigned i=0;i<rotationFinders.size();i++) rotationFinders[i]->getRotations(Rcur);
	    rotationWork[indexwork]->opMulTM(Rcur,Rcur);
	}
	
	if (useRotation) {
		for (unsigned i=0;useRotation && i<rotationFinders.size();i++) Rcur->opMulTV(systemLHVector,systemRHVector);
		this->solve(*matricesWork[indexwork], tmpVectorRotation, *systemLHVector);
		for (unsigned i=0;useRotation && i<rotationFinders.size();i++) Rcur->opMulV(systemLHVector,&tmpVectorRotation);
	} else {
		this->solve(*matricesWork[indexwork], *systemLHVector, *systemRHVector);
	}
	
	if (!solutionVecId.isNull()) {
		unsigned int offset = 0;
		v_clear(solutionVecId);
		multiVectorPeqBaseVector(solutionVecId, systemLHVector, offset);
	}
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
template<class Matrix, class Vector> template<class RMatrix, class JMatrix>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(RMatrix& result, JMatrix& J, double fact) {
  const unsigned int Jrows = J.rowSize();
  const unsigned int Jcols = J.colSize();
  if (Jcols != matricesWork[indexwork]->rowSize()) {
      serr << "LULinearSolver::addJMInvJt ERROR: incompatible J matrix size." << sendl;
      return false;
  }

  if (!Jrows) return false;

  const typename JMatrix::LineConstIterator jitend = J.end();
	  // STEP 1 : put each line of matrix Jt in the right hand term of the system
  for (typename JMatrix::LineConstIterator jit1 = J.begin(); jit1 != jitend; ++jit1) {
      int row1 = jit1->first;
      // clear the right hand term:
      systemRHVector->clear(); // currentGroup->systemMatrix->rowSize()
      //double acc = 0.0;
      for (typename JMatrix::LElementConstIterator i1 = jit1->second.begin(), i1end = jit1->second.end(); i1 != i1end; ++i1) {
	systemRHVector->add(i1->first,i1->second);       
      }

      // STEP 2 : solve the system :
      solveSystem();


      // STEP 3 : project the result using matrix J
      for (typename JMatrix::LineConstIterator jit2 = jit1; jit2 != jitend; ++jit2)
      {
	      int row2 = jit2->first;
	      double acc = 0.0;
	      for (typename JMatrix::LElementConstIterator i2 = jit2->second.begin(), i2end = jit2->second.end(); i2 != i2end; ++i2)
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
