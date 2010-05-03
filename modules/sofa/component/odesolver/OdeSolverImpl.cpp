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


#include <stdlib.h>
#include <math.h>

#ifdef SOFA_HAVE_EIGEN2
#include <Eigen/LU>
#endif

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

    if (f_printLog.getValue()) sout<<"OdeSolver::computeAcc, f = "<<f<<sendl;

    accFromF(a, f);
    projectResponse(a);
}


void OdeSolverImpl::computeContactAcc(double t, VecId a, VecId x, VecId v)
{
    MultiVector f(this, VecId::force());
    propagatePositionAndVelocity(t, x, v);
    computeContactForce(f);
    if (f_printLog.getValue()) sout<<"OdeSolver::computeContactAcc, f = "<<f<<sendl;

    accFromF(a, f);
    projectResponse(a);
}



#ifdef SOFA_HAVE_EIGEN2

  bool OdeSolverImpl::needPriorStatePropagation()
  {
    using core::componentmodel::behavior::BaseLMConstraint;
    bool needPriorPropagation=false;
    {
      helper::vector< BaseLMConstraint* > c;
      this->getContext()->get<BaseLMConstraint>(&c, core::objectmodel::BaseContext::SearchDown);     
      for (unsigned int i=0;i<c.size();++i) 
        {
          if (!c[i]->isCorrectionComputedWithSimulatedDOF())
            {
              needPriorPropagation=true;
              sout << "Propagating the State because of "<< c[i]->getName() << sendl;
              break;
            }
        }
    }
    return needPriorPropagation;
  }


  void OdeSolverImpl::solveConstraint(bool priorStatePropagation, VecId Order)
  {
    //Get the matrices through mappings
    //************************************************************
    // Update the State of the Mapped dofs                      //
    //************************************************************
    
#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::TRACE_ARGUMENT arg;
#endif
    using core::componentmodel::behavior::BaseLMConstraint ;

    sofa::simulation::MechanicalWriteLMConstraint LMConstraintVisitor;   
    BaseLMConstraint::ConstOrder orderState;
    if      (Order==VecId::dx())       
      {
        if (!constraintAcc.getValue()) return;
        orderState=BaseLMConstraint::ACC;
        if (priorStatePropagation)
          {
            simulation::MechanicalPropagateDxVisitor propagateState(Order);
            propagateState.execute(this->getContext());
          }        
        // calling writeConstraintEquations
        LMConstraintVisitor.setOrder(orderState);
        LMConstraintVisitor.setTags(getTags()).execute(this->getContext());

#ifdef SOFA_DUMP_VISITOR_INFO
        arg.push_back(std::make_pair("Order", "Acceleration"));
#endif
      }
    else if (Order==VecId::velocity()) 
      {
        if (!constraintVel.getValue()) return;
        orderState=BaseLMConstraint::VEL;
        if (priorStatePropagation)
          {
            simulation::MechanicalPropagateVVisitor propagateState(Order);
            propagateState.execute(this->getContext());
          }
        // calling writeConstraintEquations
        LMConstraintVisitor.setOrder(orderState);
        LMConstraintVisitor.setTags(getTags()).execute(this->getContext());

#ifdef SOFA_DUMP_VISITOR_INFO
        arg.push_back(std::make_pair("Order", "Velocity"));
#endif

      }
    else
      {
        if (!constraintPos.getValue()) return;
        orderState=BaseLMConstraint::POS;

        if (priorStatePropagation)
          {
            simulation::MechanicalPropagateXVisitor propagateState(Order);
            propagateState.execute(this->getContext());
          }
        // calling writeConstraintEquations
        LMConstraintVisitor.setOrder(orderState);
        LMConstraintVisitor.setTags(getTags()).execute(this->getContext());

#ifdef SOFA_DUMP_VISITOR_INFO
        arg.push_back(std::make_pair("Order", "Position"));
#endif

      }


    //************************************************************
    // Find the number of constraints                           //
    //************************************************************
    unsigned int numConstraint=0;
    const helper::vector< BaseLMConstraint* > &LMConstraints=LMConstraintVisitor.getConstraints();
    for (unsigned int mat=0;mat<LMConstraints.size();++mat)
      {
	numConstraint += LMConstraints[mat]->getNumConstraint(orderState);
      }
    if (numConstraint == 0) return; //Nothing to solve

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SolveConstraint", "LMConstraint", arg);
#endif

    if (f_printLog.getValue()) 
      {
        if (Order==VecId::dx())            sout << "Applying the constraint on the acceleration"<<sendl;
        else if (Order==VecId::velocity()) sout << "Applying the constraint on the velocity"<<sendl;
        else if (Order==VecId::position()) sout << "Applying the constraint on the position"<<sendl;
      }


#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SystemCreation");
#endif
    //Informations to build the matrices
    //Dofs to be constrained
    typedef std::set< sofa::core::componentmodel::behavior::BaseMechanicalState* >::const_iterator DofIterator;
    std::set< sofa::core::componentmodel::behavior::BaseMechanicalState* > setDofs;
    //Store the matrix M^-1.L^T for each dof in order to apply the modification of the state
    std::vector< DofToMatrix< SparseMatrixEigen > > invMass_Ltrans;
    //To Build L.M^-1.L^T, we need to know what line of the VecConst will be used
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::vector< sofa::helper::vector< unsigned int > > > indicesUsedSystem;
    //To Build L.M^-1.L^T, we need to know to what system of contraint: the offset helps to write the matrix J
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::vector< unsigned int > > offsetSystem;
    std::map< sofa::core::componentmodel::behavior::BaseMechanicalState*, sofa::helper::set< unsigned int > > dofUsed;


    unsigned int offset;
    //************************************************************
    // Gather the information from all the constraint components
    //************************************************************
    unsigned constraintOffset=0;
    for (unsigned int mat=0;mat<LMConstraints.size();++mat)
      {   
	BaseLMConstraint *constraint=LMConstraints[mat];

	sofa::helper::vector< unsigned int > indicesUsed[2];
	constraint->getIndicesUsed1(orderState, indicesUsed[0]);
	constraint->getIndicesUsed2(orderState, indicesUsed[1]);


	unsigned int currentNumConstraint=constraint->getNumConstraint(orderState);
	setDofs.insert(constraint->getSimulatedMechModel1());
	setDofs.insert(constraint->getSimulatedMechModel2());

	indicesUsedSystem[constraint->getSimulatedMechModel1()].push_back(indicesUsed[0]);
	indicesUsedSystem[constraint->getSimulatedMechModel2()].push_back(indicesUsed[1]);
	
	offsetSystem[constraint->getSimulatedMechModel1()].push_back(constraintOffset);
	offsetSystem[constraint->getSimulatedMechModel2()].push_back(constraintOffset);

	constraintOffset += currentNumConstraint;
      }


    //************************************************************
    // Build the Right Hand Term
    //************************************************************
    VectorEigen cEigen((int)numConstraint);
    buildRightHandTerm(orderState,LMConstraints, cEigen);
    //************************************************************
    // Building A=J0.M0^-1.J0^T + J1.M1^-1.J1^T + ... and M^-1.J^T 
    //************************************************************
    SparseMatrixEigen AEi((int)numConstraint,(int)numConstraint);


    std::vector< DofToMatrix< SparseMatrixEigen > >::iterator invMass;

    for (DofIterator itDofs=setDofs.begin(); itDofs!=setDofs.end();itDofs++)
      {
        sofa::core::componentmodel::behavior::BaseMechanicalState* dofs=*itDofs;
        const unsigned int dimensionDofs=dofs->getDerivDimension();
        


        //************************************************************
        //Find FixedPoints M^-1
        //Apply Constraint on the inverse of the mass matrix: should maybe need a better interface
        FullVector<double>  FixedPoints(dofs->getSize());
        for (unsigned int i=0;i<FixedPoints.size();++i) FixedPoints.set(i,1.0);
        offset=0;
        sofa::helper::vector< core::componentmodel::behavior::BaseConstraint *> listConstraintComponent;
        dofs->getContext()->get<core::componentmodel::behavior::BaseConstraint>(&listConstraintComponent, core::objectmodel::BaseContext::Local);
        //Set to zero all the particles fixed
        for (unsigned int j=0;j<listConstraintComponent.size();++j) 
          {
            listConstraintComponent[j]->applyInvMassConstraint(&FixedPoints,offset);
          }


        //************************************************************
        //Building M^-1        
        //Called only ONCE! 
	core::componentmodel::behavior::BaseMass *mass=dynamic_cast< core::componentmodel::behavior::BaseMass *>(dofs->getContext()->getMass());


        //Verify if the Big M^-1 matrix has not already been computed and stored in memory
        invMass = std::find(invMassMatrix.begin(),invMassMatrix.end(),dofs);


        if (invMass == invMassMatrix.end())
          {
            unsigned int dim=dimensionDofs;
//             if (dofs->getDerivDimension() == 6 && dofs->getCoordDimension() == 7) dim=3;


            //Build M, using blocks of [ dim x dim ] given for each particle
            SparseMatrixEigen MEigen(dofs->getSize()*dim, dofs->getSize()*dim);
            MEigen.startFill(dofs->getSize()*dim*dim);
            if (mass)
              {
                //computationM is a block corresponding the mass matrix of a particle
                FullMatrix<SReal> computationM(dim, dim);
                MatrixEigen invMEigen((int)dim,(int)dim);

                for (int i=0;i<dofs->getSize();++i)
                  {

                    if (!FixedPoints[i]) continue;
                    mass->getElementMass(i,&computationM);


                    //Translate the FullMatrix into a Eigen Matrix to invert it
                    MatrixEigen mapMEigen=Eigen::Map<MatrixEigen>(computationM[0],(int)computationM.rowSize(),(int)computationM.colSize());
                    mapMEigen.computeInverse(&invMEigen);


                    //Store into the sparse matrix the block corresponding to the inverse of the mass matrix of a particle
                    for (unsigned int r=0;r<dim;++r)
                      {
                        for (unsigned int c=0;c<dim;++c)
                          {
                            if (invMEigen(r,c) != 0)
                              MEigen.fill(i*dim+r,i*dim+c)=invMEigen(r,c);
                          }
                      }
                  }
              }
            MEigen.endFill();
            //Store the matrix in memory
            invMassMatrix.push_back( DofToMatrix< SparseMatrixEigen >(dofs,MEigen) );
            invMass = invMassMatrix.end()-1;
          }


        if (!mass) continue;

        //************************************************************
        //Building L 

         std::set< unsigned int > &usage=dofUsed[dofs];

        SparseMatrixEigen L(numConstraint,dofs->getSize()*dofs->getDerivDimension());
        L.startFill(numConstraint*(1+dofs->getSize())); //TODO: give a better estimation of non-zero coefficients

        //number of constraint expressed with the current dof
        for (unsigned int idConstraint=0;idConstraint<indicesUsedSystem[dofs].size();++idConstraint)
          {
            //Ask the dof to give the content of a list of VecConst, organized into blocks
            std::list<unsigned int > entries(indicesUsedSystem[dofs][idConstraint].begin(),indicesUsedSystem[dofs][idConstraint].end()); 
            typedef core::componentmodel::behavior::BaseMechanicalState::ConstraintBlock ConstraintBlock;
           
            //A block has as many lines as constraints for the current dof.
            std::list< ConstraintBlock > blocks=dofs->constraintBlocks( entries );
            


            const unsigned int line=offsetSystem[dofs][idConstraint];
            
            std::list< ConstraintBlock >::iterator itBlock;
            for (itBlock=blocks.begin();itBlock!=blocks.end();itBlock++)
              {                
                const ConstraintBlock &b=(*itBlock);
                const defaulttype::BaseMatrix &m=b.getMatrix(); 
                const unsigned int column=b.getColumn()*dimensionDofs;
                    
                for (unsigned int i=0;i<m.rowSize();++i)
                  { 
                    for (unsigned int j=0;j<m.colSize();++j)
                      { 
                        SReal value=m.element(i,j);
                        if (value!=0)
                          {
                            L.coeffRef(line+i, column+j) += m.element(i,j);
                            usage.insert((column+j)/dimensionDofs);
                          }
                      }
                  }
              }

            for (itBlock=blocks.begin();itBlock!=blocks.end();itBlock++)
              {
                delete itBlock->getMatrix();
              }
          }
        L.endFill();
        if (f_printLog.getValue())  sout << "Matrix L for " << dofs->getName() << "\n" << L << sendl;
        //************************************************************
        //Accumulation
        //Simple way to compute
        //const SparseMatrixEigen &invM_LtransMatrix = invMass->matrix*L.transpose();
        if (f_printLog.getValue())  sout << "Matrix M-1 for " << dofs->getName() << "\n" << invMass->matrix << sendl;

        //Taking into account that invM is block diagonal

        SparseMatrixEigen invM_LtransMatrix(L.rows(),invMass->matrix.rows());
        invM_LtransMatrix.startFill(L.nonZeros()*dimensionDofs);
        for (int k=0; k<L.outerSize(); ++k)
          {
            int accumulatingDof=-1;
            unsigned int column=0;
            std::vector< SReal > value(dimensionDofs,0);
            for (SparseMatrixEigen::InnerIterator it(L,k); it; ++it)
              {
                const unsigned int currentIndexDof = it.col()/dimensionDofs;
                const unsigned int d = it.col()%dimensionDofs;
                if (accumulatingDof < 0)
                  {
                    accumulatingDof = currentIndexDof;
                    column = it.row();
                    value.clear();
                  }
                else if (accumulatingDof != (int)currentIndexDof)
                  {
                    for (unsigned int iM=0;iM<dimensionDofs;++iM)
                      {
                        SReal finalValue=SReal(0);
                        for (unsigned int iL=0;iL<dimensionDofs;++iL)
                          {
                            finalValue += invMass->matrix.coeff(dimensionDofs*accumulatingDof+iM,dimensionDofs*accumulatingDof+iL) * value[iL];
                          }
                        invM_LtransMatrix.fill(column,dimensionDofs*accumulatingDof+iM)=finalValue;
                      }

                    accumulatingDof = currentIndexDof; 
                    column = it.row();
                    value.clear();
                  }
            
                value[d] = it.value();
              }
            if (accumulatingDof >= 0)
              {
                for (unsigned int iM=0;iM<dimensionDofs;++iM)
                  {
                    SReal finalValue=SReal(0);
                    for (unsigned int iL=0;iL<dimensionDofs;++iL)
                      {
                        finalValue += invMass->matrix.coeff(dimensionDofs*accumulatingDof+iM,dimensionDofs*accumulatingDof+iL) * value[iL];
                      }
                    invM_LtransMatrix.fill(column,dimensionDofs*accumulatingDof+iM)=finalValue;
                  }
              }
          }
        invM_LtransMatrix.endFill();

        //The simple way would be
        //invMass_Ltrans.push_back( DofToMatrix< SparseMatrixEigen >(dofs,invM_LtransMatrix) );
        //Optimized way is
        invMass_Ltrans.push_back( DofToMatrix< SparseMatrixEigen >(dofs,invM_LtransMatrix.transpose()) );

        AEi = AEi + L*invMass_Ltrans.back().matrix;
      }  


    //************************************************************
    // Solving the System using Eigen2
    //************************************************************
    if (f_printLog.getValue()) 
      {
        sout << "A= J0.M0^-1.J0^T + J1.M1^-1.J1^T + ...: "<<sendl;
        
        sout <<"\n" <<AEi << sendl;
        sout << "for a constraint: " << ""<<sendl;
        sout << "\n" << cEigen << sendl;
      }


#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SystemCreation");
#endif

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SystemSolution");
#endif

    VectorEigen LambdaEigen=VectorEigen::Zero(numConstraint);
    if (f_printLog.getValue()) sout << "Using Gauss-Seidel solution"<<sendl;


    //Convert the Sparse Matrix AEi into a Dense Matrix-> faster access to elements, and possilibity to use a direct LU solution
    MatrixEigen AEigen=MatrixEigen::Zero((int)numConstraint, (int)numConstraint);
    for (int k=0; k<AEi.outerSize(); ++k)
      for (SparseMatrixEigen::InnerIterator it(AEi,k); it; ++it)
        {
          AEigen(it.row(),it.col()) = it.value();
        }
    //-- Initialization of X, solution of the system
    bool continueIteration=true;
    unsigned int iteration=0;
    double error=0;
    for (;iteration < numIterations.getValue() && continueIteration;++iteration)
      {
        unsigned int idxConstraint=0;
        VectorEigen varEigen;
        VectorEigen previousIterationEigen;
        continueIteration=false;
        //Iterate on all the Constraint components
        for (unsigned int componentConstraint=0;componentConstraint<LMConstraints.size();++componentConstraint)
          {
            BaseLMConstraint *constraint=LMConstraints[componentConstraint];
            //Get the vector containing all the constraint stored in one component
            const std::vector< BaseLMConstraint::ConstraintGroup* > &constraintOrder=constraint->getConstraintsOrder(orderState); 

            for (unsigned int constraintEntry=0;constraintEntry<constraintOrder.size();++constraintEntry)
              {
                //-------------------------------------
                //Initialize the variables, and store X^(k-1) in previousIteration
                unsigned int numConstraintToProcess=constraintOrder[constraintEntry]->getNumConstraint();
                varEigen = VectorEigen::Zero(numConstraintToProcess);
                previousIterationEigen = VectorEigen::Zero(numConstraintToProcess);
                for (unsigned int i=0;i<numConstraintToProcess;++i)
                  {
                    previousIterationEigen(i)=LambdaEigen(idxConstraint+i);
                    LambdaEigen(idxConstraint+i)=0;
                  }
                //    operation: A.X^k --> var

                varEigen = AEigen.block(idxConstraint,0,numConstraintToProcess,numConstraint)*LambdaEigen;
                error=0;
                bool groupDesactivated=false;
                unsigned int i=0;
                for (i=0;i<numConstraintToProcess;++i)
                  {
                    //X^(k)= (c^(0)-A[c,c]*X^(k-1))/A[c,c]
                    LambdaEigen(idxConstraint+i)=(cEigen(idxConstraint+i) - varEigen(i))/AEigen(idxConstraint+i,idxConstraint+i);
                    if (constraintOrder[constraintEntry]->getConstraint(i).nature == BaseLMConstraint::UNILATERAL && LambdaEigen(idxConstraint+i) < 0) 
                      {
                        groupDesactivated=true;
                        if (f_printLog.getValue()) sout << "Constraint : " << i << " from group " << idxConstraint << " Desactivated" << sendl;
                        break;
                      }
                    error += pow(previousIterationEigen(i)-LambdaEigen(idxConstraint+i),2);			  
                  }
                //One of the Unilateral Constraint is not active anymore. We desactivate the whole group
                if (groupDesactivated)
                  {
                    for (unsigned int j=0;j<numConstraintToProcess;++j)
                      {                     
                        if (j<i) error -= pow(previousIterationEigen(j)-LambdaEigen(idxConstraint+j),2);
                        LambdaEigen(idxConstraint+j) = 0;
                        error += pow(previousIterationEigen(j)-LambdaEigen(idxConstraint+j),2);
                      }
                  }
                error = sqrt(error);
                //****************************************************************
                if (this->f_printLog.getValue()) 
                  {
                    if (f_printLog.getValue()) sout << "Error is : " <<  error << " for system of constraint " << idxConstraint<< "[" << numConstraintToProcess << "]" << "/" << AEigen.cols() 
                                                    << " between " << constraint->getSimulatedMechModel1()->getName() << " and " << constraint->getSimulatedMechModel2()->getName() << ""<<sendl;
                  }
                //****************************************************************
                //Update only if the error is higher than a threshold. If no "big changes" occured, we set: X[c]^(k) = X[c]^(k-1)
                if (error < maxError.getValue())
                  {		
                    for (unsigned int i=0;i<numConstraintToProcess;++i)
                      {
                        LambdaEigen(idxConstraint+i)=previousIterationEigen(i);
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
            if (f_printLog.getValue()) sout << "ITERATION " << iteration << " ENDED\n"<<sendl;
          }
      }
    if (iteration == numIterations.getValue())
      {
        serr << "no convergence in Gauss-Seidel"<<sendl;
        return;
      }

    if (f_printLog.getValue()) sout << "Gauss-Seidel done in " << iteration << " iterations "<<sendl;
      

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SystemSolution");
#endif


#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SystemCorrection");
#endif

    //*********************************
    //Correct States
    //*********************************
    //************************************************************
    // Updating the state vectors
    // get the displacement. deltaState = M^-1.J^T.lambda : lambda being the solution of the system
    for (DofIterator itDofs=setDofs.begin();itDofs!=setDofs.end();itDofs++)
      {
        sofa::core::componentmodel::behavior::BaseMechanicalState* dofs=*itDofs;

        core::componentmodel::behavior::BaseMass *mass=dynamic_cast< core::componentmodel::behavior::BaseMass *>(dofs->getContext()->getMass());
        if (!mass) continue;
        std::vector< DofToMatrix<SparseMatrixEigen> >::const_iterator invM_LtransMatrix = std::find( invMass_Ltrans.begin(),invMass_Ltrans.end(), dofs);            

        constraintStateCorrection(Order, dofs, invM_LtransMatrix->matrix , LambdaEigen, dofUsed[dofs]);
      }

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SystemCorrection");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SolveConstraint");
#endif
  }




  void OdeSolverImpl::buildRightHandTerm( ConstOrder Order, const helper::vector< core::componentmodel::behavior::BaseLMConstraint* > &LMConstraints, VectorEigen &c)
  {
    using core::componentmodel::behavior::BaseLMConstraint ;

    unsigned int offset=0;
    for (unsigned int mat=0;mat<LMConstraints.size();++mat)
      {
        helper::vector<SReal> correction; LMConstraints[mat]->getCorrections(Order,correction);
        for (unsigned int numC=0;numC<correction.size();++numC)
          {
            c(offset+numC)=correction[numC];
          }
        offset += correction.size();
      }
  }






  void OdeSolverImpl::constraintStateCorrection(VecId &Order, sofa::core::componentmodel::behavior::BaseMechanicalState* dofs, 
                                                const SparseMatrixEigen  &invM_Ltrans, const VectorEigen  &c, sofa::helper::set< unsigned int > &dofUsed)
  {

    //Correct Dof
    //    operation: M0^-1.J0^T.lambda -> A

   VectorEigen A = invM_Ltrans*c;
   if (f_printLog.getValue()) 
     {
       sout << "M^-1.L^T " << "\n" << invM_Ltrans << sendl;
       sout << "Lambda " << dofs->getName() << "\n" << c << sendl;
       sout << "Correction " << dofs->getName() << "\n" << A << sendl;
     }
    const unsigned int dimensionDofs=dofs->getDerivDimension();

    unsigned int offset=0;
    //In case of position correction, we need to update the velocities
    if (Order==VecId::position())
      {
	//Detect Rigid Bodies
        if (dofs->getCoordDimension() == 7 && dofs->getDerivDimension() == 6)
	  {
            VectorEigen Acorrection=VectorEigen::Zero(dofs->getSize()*(3+4));
	    //We have to transform the Euler Rotations into a quaternion
	    offset=0;

            for (int l=0;l<A.rows();l+=6)
              {
                offset=l/6;
                Acorrection(l+0+offset)=A(l+0);
                Acorrection(l+1+offset)=A(l+1);
                Acorrection(l+2+offset)=A(l+2);
	
                defaulttype::Quaternion q=defaulttype::Quaternion::createQuaterFromEuler(defaulttype::Vector3(A(l+3),A(l+4),A(l+5)));
	                       
                Acorrection(l+3+offset)=q[0];
                Acorrection(l+4+offset)=q[1];
                Acorrection(l+5+offset)=q[2];
                Acorrection(l+6+offset)=q[3];
              }
            offset=0;
            FullVector<SReal> v(Acorrection.data(),Acorrection.rows());

            if (f_printLog.getValue())  sout << "Lambda Corrected for Rigid " << "\n" << Acorrection << sendl;
            dofs->addBaseVectorToState(VecId::position(),&v,offset );

	  }
	else
	  { 
            std::set< unsigned int >::const_iterator it;
            for (it=dofUsed.begin();it!=dofUsed.end();it++)
              {
                unsigned int offset=(*it);
                FullVector<SReal> v(&(A.data()[offset*dimensionDofs]),dimensionDofs);
                dofs->addVectorToState(VecId::position(),&v,offset );
              }
	  }

          {
            const double h=1.0/getContext()->getDt();

            std::set< unsigned int >::const_iterator it;
            for (it=dofUsed.begin();it!=dofUsed.end();it++)
              {
                unsigned int offset=(*it);
                FullVector<SReal> v(&(A.data()[offset*dimensionDofs]),dimensionDofs);
                for (unsigned int i=0;i<dimensionDofs;++i) v[i]*=h;
                dofs->addVectorToState(VecId::velocity(),&v,offset );
              }
          }

      }
    else
      { 	
        std::set< unsigned int >::const_iterator it;
        for (it=dofUsed.begin();it!=dofUsed.end();it++)
          {
            unsigned int offset=(*it);
            FullVector<SReal> v(&(A.data()[offset*dimensionDofs]),dimensionDofs);
            dofs->addVectorToState(Order,&v,offset );
          }

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
