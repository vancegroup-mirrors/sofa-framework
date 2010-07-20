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
ParallelMatrixLinearSolver<Matrix,Vector>::ParallelMatrixLinearSolver()
: multiGroup( initData( &multiGroup, false, "multiGroup", "activate multiple system solve, one for each child node" ) )
, useWarping( initData( &useWarping, false, "useWarping", "use Warping around the solver" ) )
, currentGroup(&defaultGroup)
{}

template<class Matrix, class Vector>
ParallelMatrixLinearSolver<Matrix,Vector>::~ParallelMatrixLinearSolver()
{
    //if (systemMatrix) deleteMatrix(systemMatrix);
    //if (systemRHVector) deleteVector(systemRHVector);
    //if (systemLHVector) deleteVector(systemLHVector);
}



template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::createGroups() {
    simulation::Node* root = dynamic_cast<simulation::Node*>(this->getContext());
    //defaultGroup.node = root;
    nodeMap.clear();
    writeNodeMap.clear();
    for (GroupDataMapIter it = gData.begin(), itend = gData.end(); it != itend; ++it)
        it->second.systemSize = 0;
    if (isMultiGroup())
    {
        for (unsigned int g=0;g<root->child.size();++g)
        {
            simulation::Node* n = root->child[g];
            nodeMap[n] = 0.0;
        }

        double dim = 0;
        simulation::MechanicalGetDimensionVisitor(&dim).setNodeMap(&nodeMap).execute(root);

        groups.clear();

        for (unsigned int g=0;g<root->child.size();++g)
        {
            simulation::Node* n = root->child[g];
            double gdim = nodeMap[ n ];
            if (gdim <= 0) continue;
            groups.push_back(n);
            gData[n].systemSize = (int)gdim;
            dim += gdim;
        }

        defaultGroup.systemSize = (int)dim;

        // set nodemap to default (i.e. factor 1 for all non empty groups
        nodeMap.clear();
        writeNodeMap.clear();
        for (unsigned int g=0;g<groups.size();++g)
        {
            nodeMap[ groups[g] ] = 1.0;
            writeNodeMap[ groups[g] ] = 0.0;
        }
    }
    else
    {
        groups.clear();
        double dim = 0;
        simulation::MechanicalGetDimensionVisitor(&dim).execute(root);
        defaultGroup.systemSize = (int)dim;
    }
    currentNode = root;
    currentGroup = &defaultGroup;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resetSystem()
{
    for (unsigned int g=0, nbg = isMultiSolve() ? 1 : getNbGroups(); g < nbg; ++g) { if (!isMultiSolve()) setGroup(g);
    if (!frozen)
    {
        if (currentGroup->systemMatrix) currentGroup->systemMatrix->clear();
        currentGroup->needInvert = true;
    }
    if (currentGroup->systemRHVector) currentGroup->systemRHVector->clear();
    if (currentGroup->systemLHVector) currentGroup->systemLHVector->clear();
    currentGroup->solutionVecId = VecId();
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::resizeSystem(int n)
{
    if (!frozen)
    {
        if (!currentGroup->systemMatrix) currentGroup->systemMatrix = createMatrix();
        currentGroup->systemMatrix->resize(n,n);
    }
    if (!currentGroup->systemRHVector) currentGroup->systemRHVector = createVector();
    currentGroup->systemRHVector->resize(n);
    if (!currentGroup->systemLHVector) currentGroup->systemLHVector = createVector();
    currentGroup->systemLHVector->resize(n);
    currentGroup->needInvert = true;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemMBKMatrix(double mFact, double bFact, double kFact) {
    getRotations(Rinv);
  
    createGroups();
    for (unsigned int g=0, nbg = isMultiSolve() ? 1 : getNbGroups(); g < nbg; ++g) { 
      if (!isMultiSolve()) setGroup(g);
      if (!this->frozen) {
	  unsigned int nbRow=0, nbCol=0;
	  //MechanicalGetMatrixDimensionVisitor(nbRow, nbCol).execute( getContext() );
	  this->getMatrixDimension(&nbRow, &nbCol);
	  resizeSystem(nbRow);
	  currentGroup->systemMatrix->clear();
	  unsigned int offset = 0;
	  //MechanicalAddMBK_ToMatrixVisitor(currentGroup->systemMatrix, mFact, bFact, kFact, offset).execute( getContext() );
	  this->addMBK_ToMatrix(currentGroup->systemMatrix, mFact, bFact, kFact, offset);
      }
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::getRotations(Vector & R) {
    if (! useWarping.getValue()) return;
    
    unsigned int size = currentGroup->systemSize; // dont know how it works if there is multigroup
    R.resize(size*9);
    
    if (rotationFinders.empty()) {
	serr << "No rotation defined : use Identity !!";
	for(unsigned int k = 0; k < size; k++)	{
		R[k*9] = R[k*9+4] = R[k*9+8] = 1.0f;
		R[k*9+1] = R[k*9+2] = R[k*9+3] = R[k*9+5] = R[k*9+6] = R[k*9+7] = 0.0f;
	}
    } else {
        for (unsigned i=0;i<rotationFinders.size();i++) rotationFinders[i]->getRotations(&R);
    }
    
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemRHVector(VecId v)
{
    for (unsigned int g=0, nbg = isMultiSolve() ? 1 : getNbGroups(); g < nbg; ++g) { 
	if (!isMultiSolve()) setGroup(g);
	unsigned int offset = 0;
	//MechanicalMultiVector2BaseVectorVisitor(v, systemRHVector, offset).execute( getContext() );
	this->multiVector2BaseVector(v, currentGroup->systemRHVector, offset);
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::setSystemLHVector(VecId v)
{
    for (unsigned int g=0, nbg = isMultiSolve() ? 1 : getNbGroups(); g < nbg; ++g) { if (!isMultiSolve()) setGroup(g);
    currentGroup->solutionVecId = v;
    unsigned int offset = 0;
    //MechanicalMultiVector2BaseVectorVisitor(v, systemLHVector, offset).execute( getContext() );
    this->multiVector2BaseVector(v, currentGroup->systemLHVector, offset);
    }
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::solveSystem()
{
    for (unsigned int g=0, nbg = isMultiSolve() ? 1 : getNbGroups(); g < nbg; ++g) { if (!isMultiSolve()) setGroup(g);
    if (currentGroup->needInvert)
    {
        this->invert(*currentGroup->systemMatrix);
        currentGroup->needInvert = false;
    }
    this->solve(*currentGroup->systemMatrix, *currentGroup->systemLHVector, *currentGroup->systemRHVector);
    if (!currentGroup->solutionVecId.isNull())
    {
        unsigned int offset = 0;
        v_clear(currentGroup->solutionVecId);
        multiVectorPeqBaseVector(currentGroup->solutionVecId, currentGroup->systemLHVector, offset);
    }
    }
}

template<class Matrix, class Vector>
Vector* ParallelMatrixLinearSolver<Matrix,Vector>::createVector()
{
    return new Vector;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::deleteVector(Vector* v)
{
    delete v;
}

template<class Matrix, class Vector>
Matrix* ParallelMatrixLinearSolver<Matrix,Vector>::createMatrix()
{
    return new Matrix;
}

template<class Matrix, class Vector>
void ParallelMatrixLinearSolver<Matrix,Vector>::deleteMatrix(Matrix* v)
{
    delete v;
}

template<class TMatrix, class TVector>
void ParallelMatrixLinearSolver<TMatrix,TVector>::init() {
    sofa::core::objectmodel::BaseContext * c = this->getContext();

    c->get<sofa::component::misc::BaseRotationFinder >(&rotationFinders, sofa::core::objectmodel::BaseContext::Local);
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
///
/// TODO : put this implementation in ParallelMatrixLinearSolver class - fix problems mith Scattered Matrix
template<class Matrix, class Vector> template<class RMatrix, class JMatrix>
bool ParallelMatrixLinearSolver<Matrix,Vector>::addJMInvJt(RMatrix& result, JMatrix& J, double fact)
{
  const unsigned int Jrows = J.rowSize();
  const unsigned int Jcols = J.colSize();
  if (Jcols != currentGroup->systemMatrix->rowSize())
  {
      serr << "LULinearSolver::addJMInvJt ERROR: incompatible J matrix size." << sendl;
      return false;
  }

  if (!Jrows) return false;

  const typename JMatrix::LineConstIterator jitend = J.end();
	  // STEP 1 : put each line of matrix Jt in the right hand term of the system
  for (typename JMatrix::LineConstIterator jit1 = J.begin(); jit1 != jitend; ++jit1) {
      int row1 = jit1->first;
      // clear the right hand term:
      currentGroup->systemRHVector->clear(); // currentGroup->systemMatrix->rowSize()
      //double acc = 0.0;
      for (typename JMatrix::LElementConstIterator i1 = jit1->second.begin(), i1end = jit1->second.end(); i1 != i1end; ++i1) {
	currentGroup->systemRHVector->add(i1->first,i1->second);       
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
		      acc += val2 * currentGroup->systemLHVector->element(col2);
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
