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
#include <sofa/component/odesolver/OdeSolverImpl.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/MechanicalMatrixVisitor.h>
#include <sofa/simulation/common/MechanicalVPrintVisitor.h>
#include <sofa/simulation/common/VelocityThresholdVisitor.h>
#include <sofa/core/componentmodel/behavior/LinearSolver.h>

#include <sofa/defaulttype/Quat.h>

#ifdef SOFA_HAVE_LAPACK
#include <sofa/component/linearsolver/LapackOperations.h>
#endif

#include <stdlib.h>
#include <math.h>

namespace sofa
{

namespace component
{

namespace odesolver
{

  using linearsolver::FullVector;
  using linearsolver::FullMatrix;
  OdeSolverImpl::OdeSolverImpl()
  {
  }

void OdeSolverImpl::init()
{
  OdeSolver::init();
  SolverImpl::init();
  reinit();
}

void OdeSolverImpl::propagatePositionAndVelocity(double t, VecId x, VecId v)
{
  simulation::MechanicalPropagatePositionAndVelocityVisitor(t,x,v).setTags(getTags()).execute( getContext() );
}

void OdeSolverImpl::computeAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, VecId::force());
    propagatePositionAndVelocity(t, x, v);
    computeForce(f);
    if( this->f_printLog.getValue()==true ){
        serr<<"OdeSolver::computeAcc, f = "<<f<<sendl;
    }

    accFromF(a, f);
    projectResponse(a);
}


void OdeSolverImpl::computeContactAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, VecId::force());
    propagatePositionAndVelocity(t, x, v);
    computeContactForce(f);
    if( this->f_printLog.getValue()==true ){
        serr<<"OdeSolver::computeContactAcc, f = "<<f<<sendl;
    }

    accFromF(a, f);
    projectResponse(a);
}



#ifdef SOFA_HAVE_LAPACK
  void OdeSolverImpl::applyConstraints()
  {    

    
    if (constraintPos.getValue())
      {

	bool propagateCorrectOfPositionOnVelocity = !constraintVel.getValue();
	solveConstraint(VecId::position(), propagateCorrectOfPositionOnVelocity);
// 	if (propagateCorrectOfPositionOnVelocity)
// 	  {
// 	    simulation::MechanicalPropagatePositionAndVelocityVisitor propPosAndVelocity;
// 	    propPosAndVelocity.setTags(getTags()).execute(this->getContext());
// 	  }
// 	else
// 	  {
// 	    simulation::MechanicalPropagatePositionVisitor propPos;
// 	    propPos.setTags(getTags()).execute(this->getContext());
// 	  }
      }
    if (constraintVel.getValue())
      {
	solveConstraint(VecId::velocity());
      }
  }
  void OdeSolverImpl::solveConstraint(VecId Id, bool propagateVelocityFromPosition)
  {
    if (this->f_printLog.getValue())
      {
	if (Id==VecId::dx())            serr << "Applying the constraint on the acceleration"<<sendl;
	else if (Id==VecId::velocity()) serr << "Applying the constraint on the velocity"<<sendl;
	else if (Id==VecId::position()) serr << "Applying the constraint on the position"<<sendl;
      }

    //Get the matrices through mappings

    // mechanical action executed from root node to propagate the constraints
    simulation::MechanicalResetConstraintVisitor().setTags(getTags()).execute(this->getContext());
    // calling writeConstraintEquations
    sofa::simulation::MechanicalAccumulateLMConstraint LMConstraintVisitor;
    LMConstraintVisitor.setTags(getTags()).execute(this->getContext());


    using core::componentmodel::behavior::BaseLMConstraint ;
    BaseLMConstraint::ConstId idxState;
    if      (Id==VecId::dx())       idxState=BaseLMConstraint::ACC;
    else if (Id==VecId::velocity()) idxState=BaseLMConstraint::VEL;
    else                            idxState=BaseLMConstraint::POS;

    //************************************************************
    // Find the number of constraints                           //
    //************************************************************
    unsigned int numConstraint=0;
    for (unsigned int mat=0;mat<LMConstraintVisitor.numConstraintDatas();++mat)
      {
	ConstraintData& constraint=LMConstraintVisitor.getConstraint(mat);
	numConstraint += constraint.data->getNumConstraint(idxState);
      }
    if (numConstraint == 0) return; //Nothing to solve

    //Right Hand term
    FullVector<double>  c;
    c.resize(numConstraint);

    //Left Hand term: J0.M^-1.J0^T + J1.M1^-1.J1^T + ... 
    FullMatrix<double>  A;
    A.resize(numConstraint,numConstraint);	


    //Informations to build the matrices
    //Dofs to be constrained
    std::set< sofa::core::componentmodel::behavior::BaseMechanicalState* > setDofs;
    //Store the matrix M^-1.J^T for each dof in order to apply the modification of the state
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, FullMatrix<double>  > invM_Jtrans;
    //To Build J.M^-1.J^T, we need to know what line of the VecConst will be used
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::vector< sofa::helper::vector< unsigned int > > > indicesUsedSystem;
    //To Build J.M^-1.J^T, we need to know to what system of contraint: the offset helps to write the matrix J
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::vector< unsigned int > > offsetSystem;
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::vector< double > > factorSystem;


    unsigned int offset;
    //************************************************************
    // Gather the information from all the constraint components
    //************************************************************
    unsigned constraintOffset=0;
    for (unsigned int mat=0;mat<LMConstraintVisitor.numConstraintDatas();++mat)
      {
	ConstraintData& constraint=LMConstraintVisitor.getConstraint(mat);

	sofa::helper::vector< unsigned int > indicesUsed[2];
	constraint.data->getIndicesUsed(idxState, indicesUsed[0], indicesUsed[1]);


	unsigned int currentNumConstraint=constraint.data->getNumConstraint(idxState);
	setDofs.insert(constraint.independentMState[0]);
	setDofs.insert(constraint.independentMState[1]);

	indicesUsedSystem[constraint.independentMState[0] ].push_back(indicesUsed[0]);
	indicesUsedSystem[constraint.independentMState[1] ].push_back(indicesUsed[1]);
	    
	offsetSystem[constraint.independentMState[0] ].push_back(constraintOffset);
	offsetSystem[constraint.independentMState[1] ].push_back(constraintOffset);

	factorSystem[constraint.independentMState[0] ].push_back( 1.0);
	factorSystem[constraint.independentMState[1] ].push_back(-1.0);

	constraintOffset += currentNumConstraint;
      }


    //************************************************************
    // Build the Right Hand Term
    //************************************************************
    buildRightHandTerm(Id, LMConstraintVisitor, c);


    //************************************************************
    // Building A=J0.M0^-1.J0^T + J1.M1^-1.J1^T + ... and M^-1.J^T 
    //************************************************************
    std::set< sofa::core::componentmodel::behavior::BaseMechanicalState* >::const_iterator itDofs;

    for (itDofs=setDofs.begin();itDofs!=setDofs.end();itDofs++)
      {
	sofa::core::componentmodel::behavior::BaseMechanicalState* dofs=*itDofs;
	core::componentmodel::behavior::BaseMass *mass=dynamic_cast< core::componentmodel::behavior::BaseMass *>(dofs->getContext()->getMass());
	FullMatrix<double>  &M=invM_Jtrans[dofs];
	    
	if (mass)
	  {
	    //Apply Constraint on the inverse of the mass matrix: should maybe need a better interface
	    FullVector<double>  FixedPoints(dofs->getSize());
	    for (unsigned int i=0;i<FixedPoints.size();++i) FixedPoints.set(i,1.0);
	    offset=0;
	    sofa::helper::vector< core::componentmodel::behavior::BaseConstraint *> listConstraintComponent;
	    dofs->getContext()->get<core::componentmodel::behavior::BaseConstraint>(&listConstraintComponent);
	    for (unsigned int j=0;j<listConstraintComponent.size();++j) listConstraintComponent[j]->applyInvMassConstraint(&FixedPoints,offset);

	    //Compute M^-1.J^T in M, and accumulate J.M^-1.J^T in A
	    mass->buildSystemMatrix(M, A,
				    indicesUsedSystem[dofs],
				    factorSystem[dofs],
				    offsetSystem[dofs],
				    FixedPoints);
	  }
	if (this->f_printLog.getValue())
	  {
	    sout << "Matrix M^-1.J^T " << dofs->getName() << " "<<sendl;
	    printMatrix(invM_Jtrans[dofs]);
	  }	    
      }


    if (this->f_printLog.getValue())
      {
	serr << "A= J0.M0^-1.J0^T + J1.M1^-1.J1^T + ...: "<<sendl;
	printMatrix(A);
	serr << "for a constraint: " << ""<<sendl;
	printVector(c);
      }

    //************************************************************
    // Solving the System using LAPACK
    //************************************************************

    if (constraintResolution.getValue())
      {
	if (this->f_printLog.getValue()) serr << "Using Gauss-Seidel resolution"<<sendl;

	//-- Initialization of X, solution of the system
	FullVector<double>  X; X.resize(A.colSize());
	bool continueIteration=true;
	unsigned int iteration=0;
	for (;iteration < numIterations.getValue() && continueIteration;++iteration)
	  {
	    unsigned int idxConstraint=0;
	    FullVector<double>  var;
	    FullVector<double>  previousIteration;
	    continueIteration=false;
	    //Iterate on all the Constraint components
	    for (unsigned int componentConstraint=0;componentConstraint<LMConstraintVisitor.numConstraintDatas();++componentConstraint)
	      {
		ConstraintData& constraint=LMConstraintVisitor.getConstraint(componentConstraint);
		//Get the vector containing all the constraint stored in one component
		std::vector< BaseLMConstraint::constraintGroup > constraintId; 
		constraint.data->getConstraintsId(idxState, constraintId);
		for (unsigned int constraintEntry=0;constraintEntry<constraintId.size();++constraintEntry)
		  {
		    //-------------------------------------
		    //Initialize the variables, and store X^(k-1) in previousIteration
		    unsigned int numConstraintToProcess=constraintId[constraintEntry].getNumConstraint();
		    var.resize(numConstraintToProcess); var.clear();
		    previousIteration.resize(numConstraintToProcess);
		    for (unsigned int i=0;i<numConstraintToProcess;++i)
		      {
			previousIteration.set(i,X.element(idxConstraint+i));
			X.set(idxConstraint+i,0);
		      }
		    //    operation: A.X^k --> var		   
		    double alpha=1,beta=0;
		    linearsolver::applyLapackDGEMV( A.ptr()+(idxConstraint*numConstraint), false, X.ptr(), var.ptr(),
				      alpha, beta,
				      numConstraintToProcess, numConstraint);
		    double error=0;
		    for (unsigned int i=0;i<numConstraintToProcess;++i)
		      {
			//X^(k)= (c^(0)-A[c,c]*X^(k-1))/A[c,c]
			if (A.element(idxConstraint+i,idxConstraint+i) < 0.00001) 
			  {
			    serr << "Diagonal almost equal to Zero: constraint [" << idxConstraint+i << ","
				      << constraint.independentMState[0]->getName() << ","
				      << constraint.independentMState[1]->getName() << "]"
				      <<" ignored (value=" << A.element(idxConstraint+i,idxConstraint+i) << ")"<<sendl;
// 			    printMatrix(A);	
			  }
			else
			  {
			    X.set(idxConstraint+i,(c.element(idxConstraint+i) - var.element(i))/A.element(idxConstraint+i,idxConstraint+i));
			    error += pow(previousIteration.element(i)-X.element(idxConstraint+i),2);
			  }
		      }

		    error = sqrt(error);
		    //****************************************************************
		    //DEBUG!!!!!!!!!!!!!!!!!!
		    if (this->f_printLog.getValue()) 
		      {
			serr << "Error is : " <<  error << " for constraint " << idxConstraint<< "[" << numConstraintToProcess << "]" << "/" << A.colSize() 
				  << " between " << constraint.independentMState[0]->getName() << " and " << constraint.independentMState[1]->getName() << ""<<sendl;
			printVector(X);
		      }
		    //****************************************************************
		    //Update only if the error is higher than a threshold. If no "big changes" occured, we set: X[c]^(k) = X[c]^(k-1)
		    if (error < maxError.getValue())
		      {			
			for (unsigned int i=0;i<numConstraintToProcess;++i)
			  {
			    X.set(idxConstraint+i, previousIteration.element(i));
			  }
		      }
		    else
		      {
			continueIteration=true;
		      }
		    idxConstraint+=numConstraintToProcess;
		  }
	      } 
	    if (this->f_printLog.getValue()) 
 	      {
		if (iteration == numIterations.getValue()) serr << "NO CONVERGENCE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<sendl;

		serr << "ITERATION " << iteration << " ENDED\n"<<sendl;
		printVector(X);
 	      }
	  }
	if (this->f_printLog.getValue()) serr << "Gauss-Seidel done in " << iteration << " iterations "<<sendl;
	//*********************************
	//Correct States
	//*********************************
	for (itDofs=setDofs.begin();itDofs!=setDofs.end();itDofs++)
	  {		    
	    sofa::core::componentmodel::behavior::BaseMechanicalState* dofs=*itDofs;
	    constraintStateCorrection(Id, dofs, invM_Jtrans[dofs], X, propagateVelocityFromPosition);
	  }
	return;
      }
    else
      {
	// 	    if (this->f_printLog.getValue()) serr << "Using direct LU resolution"<<sendl;
	//Third Operation: solve  (J0.M0^-1.J0^T + J1.M^-1.J1^T).F = C
		  FullVector<double> X;
		  X.resize(c.size());
	linearsolver::applyLapackDGESV( A[0], X.ptr(), c.ptr(), numConstraint, 1 );

	 	 //   if (this->f_printLog.getValue())
	 	 //     {
	 		//serr << "Final Result lambda"<<sendl;
	  	//	printVector(X);
	   // 
	   // 
	 		//serr << "------------------------------------------------\n"<<sendl;
	 	 //     }

	//************************************************************
	// Updating the state vectors
	// get the displacement. deltaState = M^-1.J^T.lambda : lambda being the solution of the system
	for (itDofs=setDofs.begin();itDofs!=setDofs.end();itDofs++)
	  {
	    sofa::core::componentmodel::behavior::BaseMechanicalState* dofs=*itDofs;
	    constraintStateCorrection(Id, dofs, invM_Jtrans[dofs], X, propagateVelocityFromPosition);
	  }
      }
  }






  void OdeSolverImpl::buildRightHandTerm(VecId &Id, sofa::simulation::MechanicalAccumulateLMConstraint &LMConstraintVisitor, FullVector<double>  &c)
  {
    using core::componentmodel::behavior::BaseLMConstraint ;
    BaseLMConstraint::ConstId idxState;
    if      (Id==VecId::dx())       idxState=BaseLMConstraint::ACC;
    else if (Id==VecId::velocity()) idxState=BaseLMConstraint::VEL;
    else                            idxState=BaseLMConstraint::POS;

    unsigned constraintOffset=0;
    for (unsigned int mat=0;mat<LMConstraintVisitor.numConstraintDatas();++mat)
      {
	ConstraintData& constraint=LMConstraintVisitor.getConstraint(mat);

	sofa::helper::vector< unsigned int > indicesUsed[2];
	std::vector<double> expectedValues;
	std::vector<BaseLMConstraint::ValueId> expectedValuesType;
	constraint.data->getIndicesUsed(idxState, indicesUsed[0], indicesUsed[1]);
	constraint.data->getExpectedValues(idxState, expectedValues);
	constraint.data->getExpectedValuesType(idxState, expectedValuesType);

	unsigned int currentNumConstraint=expectedValues.size();

	//************************************************************
	//Building Right hand term
	FullVector<double>  V[2]; V[0].resize(c.size()); V[1].resize(c.size());
	//Different type of "expected values" exist: 
	//the one called FINAL. It means, the value of the constraint (zero in velocity, ...). It needs to compute the current state of the dof, in order to build the right hand term.
	//Another is CORRECTION. It already knows the value of the right hand term. Often used in Constraint dealing with position. If we want to constrain two dofs to remain at a distance d, we put directly d in the right hand term. 
	bool needComputeCurrentState=false;

	for (unsigned int i=0;i<currentNumConstraint;++i)
	  {
	    if (expectedValuesType[i] == BaseLMConstraint::FINAL) { needComputeCurrentState=true; break;}
	  }
	if (needComputeCurrentState)
	  {
	    constraint.independentMState[0]->computeConstraintProjection(indicesUsed[0], Id, V[0],constraintOffset);
	    constraint.independentMState[1]->computeConstraintProjection(indicesUsed[1], Id, V[1],constraintOffset);		  
	  }

	for (unsigned int i=constraintOffset;i<constraintOffset+currentNumConstraint;++i)
	  {
	    unsigned int idxConstraint=i-constraintOffset;
	    switch(expectedValuesType[idxConstraint])
	      {
	      case  BaseLMConstraint::FINAL:
		c.add(i,(V[1].element(i)-V[0].element(i)) - expectedValues[idxConstraint]);
		break;
	      case  BaseLMConstraint::FACTOR:
		c.add(i,(V[1].element(i)-V[0].element(i))*(expectedValues[idxConstraint]));
		break;
	      case  BaseLMConstraint::CORRECTION:
		c.add(i,(-expectedValues[idxConstraint]));
		break;
	      }
	  }

	constraintOffset += currentNumConstraint;
      }
  }






  void OdeSolverImpl::constraintStateCorrection(VecId &Id, sofa::core::componentmodel::behavior::BaseMechanicalState* dofs, 
						FullMatrix<double>  &invM_Jtrans, FullVector<double>  &c, bool propageVelocityChange)
  {
    double alpha=1,beta=0;
    if (dofs->getContext()->getMass() == NULL) return;
    FullVector<double>  A; A.resize(invM_Jtrans.rowSize());
    //Correct Dof
    //    operation: M0^-1.J0^T.lambda -> A
    linearsolver::applyLapackDGEMV( invM_Jtrans, false, c, A, alpha, beta);

    //In case of position correction, we need to update the velocities
    if (Id==VecId::position())
      {
	//Detect Rigid Bodies
	if ((unsigned int)(dofs->getSize()*(3+3)) == A.size())
	  {
		    
	    FullVector<double>  Acorrection; Acorrection.resize(dofs->getSize()*(3+4));
	    //We have to transform the Euler Rotations into a quaternion
	    unsigned int offset=0;
	    for (unsigned int l=0;l<A.size();l+=6)
	      {
		offset=l/6;
		Acorrection.set(l+0+offset,A.element(l+0));
		Acorrection.set(l+1+offset,A.element(l+1));
		Acorrection.set(l+2+offset,A.element(l+2));

		defaulttype::Quaternion q=defaulttype::Quaternion::createQuaterFromEuler(defaulttype::Vector3(A.element(l+3),A.element(l+4),A.element(l+5)));
			
		Acorrection.set(l+3+offset,q[0]);
		Acorrection.set(l+4+offset,q[1]);
		Acorrection.set(l+5+offset,q[2]);
		Acorrection.set(l+6+offset,q[3]);
	      }

	    if (this->f_printLog.getValue())
	      {
		serr << "delta RigidState : " << dofs->getName() << ""<<sendl;  printVector(Acorrection);
	      }
	    offset=0;
	    dofs->addBaseVectorToState(VecId::position(),&Acorrection,offset );
	  }
	else
	  {

	    if (this->f_printLog.getValue())
	      {
		serr << "delta State : " << dofs->getName() << ""<<sendl;  printVector(A);
	      }
	    unsigned int offset=0;
	    dofs->addBaseVectorToState(VecId::position(),&A,offset );
	  }

	if (propageVelocityChange)
	  {
	    double h=1.0/getContext()->getDt();
	    for (unsigned int i=0;i<A.size();++i) A[i]*=h;
		    
	    unsigned int offset=0;
	    dofs->addBaseVectorToState(VecId::velocity(),&A,offset );
	  }
      }
    else
      { 
		
	if (this->f_printLog.getValue())
	  {
	    serr << "delta State : " << dofs->getName() << ""<<sendl; printVector(A);
	  }

	unsigned int offset=0;
	dofs->addBaseVectorToState(Id,&A,offset );
      }	  
  }
#endif








using sofa::core::componentmodel::behavior::LinearSolver;
using sofa::core::objectmodel::BaseContext;

void OdeSolverImpl::m_resetSystem()
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->resetSystem();
}

void OdeSolverImpl::m_setSystemMBKMatrix(double mFact, double bFact, double kFact)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemMBKMatrix(mFact, bFact, kFact);
}

void OdeSolverImpl::m_setSystemRHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemRHVector(v);
}

void OdeSolverImpl::m_setSystemLHVector(VecId v)
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->setSystemLHVector(v);
}

void OdeSolverImpl::m_solveSystem()
{
  LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
//   std::cerr << getContext()<< " " << getContext()->getName() << " !!!!!! LinearSolver : " << this << "\n";
//   LinearSolver* s;
//   getContext()->get(s,BaseContext::SearchDown);
//   std::vector<core::objectmodel::BaseObject*> listObject;
//   getContext()->get<core::objectmodel::BaseObject>(&listObject, core::objectmodel::BaseContext::Local);
//   for (unsigned int i=0;i<listObject.size();++i) std::cerr << listObject[i]->getName() << "@" << listObject[i] << "\n";

//   std::vector<LinearSolver*> listLinear;
//   getContext()->get<LinearSolver>(&listLinear, core::objectmodel::BaseContext::Local);
//   for (unsigned int i=0;i<listLinear.size();++i) std::cerr << "LinearSolver " << listLinear[i]->getName() << "@" << listLinear[i] << "\n";
//   std::cerr << "S : " << s << "\n";
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    s->solveSystem();
}

void OdeSolverImpl::m_print( std::ostream& out )
{
    LinearSolver* s = getContext()->get<LinearSolver>(this->getTags(), BaseContext::SearchDown);
    if (!s)
    {
        serr << "ERROR: "<<this->getClassName()<<" requires a LinearSolver."<<sendl;
        return;
    }
    defaulttype::BaseMatrix* m = s->getSystemBaseMatrix();
    if (!m) return;
    //out << *m;
    int ny = m->rowSize();
    int nx = m->colSize();
    out << "[";
    for (int y=0;y<ny;++y)
    {
        out << "[";
        for (int x=0;x<nx;x++)
            out << ' ' << m->element(x,y);
        out << "]";
    }
    out << "]";
}

} // namespace odesolver

} // namespace component

} // namespace sofa
