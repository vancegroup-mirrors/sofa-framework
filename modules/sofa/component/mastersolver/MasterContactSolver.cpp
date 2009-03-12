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
#include <sofa/component/mastersolver/MasterContactSolver.h>

#include <sofa/simulation/common/AnimateVisitor.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/SolveVisitor.h>

#include <sofa/helper/LCPcalc.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/thread/CTime.h>
#include <math.h>
#include <iostream>




namespace sofa
{

namespace component
{

namespace odesolver
{


LCP::LCP(unsigned int mxC) : maxConst(mxC), tol(0.00001), numItMax(1000), useInitialF(true), mu(0.0), dim(0), lok(false)
{
	W.resize(maxConst,maxConst);
    dFree.resize(maxConst);
    f.resize(2*maxConst+1);
}

LCP::~LCP()
{
}

void LCP::reset(void)
{
	W.clear();
	W.clear();
	dFree.clear();
}



using namespace sofa::defaulttype;
using namespace helper::system::thread;
using namespace core::componentmodel::behavior;




#define MAX_NUM_CONSTRAINTS 1024
//#define DISPLAY_TIME

MasterContactSolver::MasterContactSolver()
:displayTime(initData(&displayTime, false, "displayTime","Display time for each important step of MasterContactSolver."))
,initial_guess(initData(&initial_guess, true, "initial_guess","activate LCP results history to improve its resolution performances."))
,build_lcp(initData(&build_lcp, true, "build_lcp", "LCP is not fully built to increase performance in some case."))
,tol( initData(&tol, 0.001, "tolerance", ""))
,maxIt( initData(&maxIt, 1000, "maxIt", ""))
,mu( initData(&mu, 0.6, "mu", ""))
, constraintGroups( initData(&constraintGroups, "group", "list of ID of groups of constraints to be handled by this solver.") )
,_mu(0.6)
, lcp1(MAX_NUM_CONSTRAINTS)
, lcp2(MAX_NUM_CONSTRAINTS)
, _W(&lcp1.W)
, lcp(&lcp1)
, _dFree(&lcp1.dFree)
, _result(&lcp1.f)
{
	_numConstraints = 0;
	_mu = 0.0;
	constraintGroups.beginEdit()->insert(0);
	constraintGroups.endEdit();

	_numPreviousContact=0;
	_PreviousContactList = (contactBuf *)malloc(MAX_NUM_CONSTRAINTS * sizeof(contactBuf));
	_cont_id_list = (long *)malloc(MAX_NUM_CONSTRAINTS * sizeof(long));
	
	_Wdiag = new SparseMatrix<double>();
}

void MasterContactSolver::init()
{
    getContext()->get<core::componentmodel::behavior::BaseConstraintCorrection>(&constraintCorrections, core::objectmodel::BaseContext::SearchDown);
}

void MasterContactSolver::build_LCP()
{
	_numConstraints = 0;
//sout<<" accumulateConstraint "  <<sendl;

	// mechanical action executed from root node to propagate the constraints
	simulation::MechanicalResetConstraintVisitor().execute(context);
	simulation::MechanicalAccumulateConstraint(_numConstraints).execute(context);
	_mu = mu.getValue();


	//sout<<" accumulateConstraint_done "  <<sendl;

	if (_numConstraints > MAX_NUM_CONSTRAINTS)
	{
		serr<<sendl<<"Error in MasterContactSolver, maximum number of contacts exceeded, "<< _numConstraints/3 <<" contacts detected"<<sendl;
		exit(-1);
	}

	lcp->getMu() = _mu;

    _dFree->resize(_numConstraints);
    _W->resize(_numConstraints,_numConstraints);

	MechanicalGetConstraintValueVisitor(_dFree).execute(context);
//	simulation::MechanicalComputeComplianceVisitor(_W).execute(context);

    if (this->f_printLog.getValue()) sout<<"MasterContactSolver: "<<_numConstraints<<" constraints, mu = "<<_mu<<sendl;

	//sout<<" computeCompliance in "  << constraintCorrections.size()<< " constraintCorrections" <<sendl;

    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->getCompliance(_W);
	}
	//sout<<" computeCompliance_done "  <<sendl;

	if (initial_guess.getValue())
	{
		MechanicalGetContactIDVisitor(_cont_id_list).execute(context);
		computeInitialGuess();
	}
}

/// build_problem_info
/// When the LCP or the NLCP is not fully built, the  diagonal blocks of the matrix are still needed for the resolution
/// This function ask to the constraintCorrection classes to build this diagonal blocks
void MasterContactSolver::build_problem_info()
{

	// debug
	//std::cout<<" accumulateConstraint "  <<std::endl;
	_numConstraints = 0;
	// mechanical action executed from root node to propagate the constraints
	simulation::MechanicalResetConstraintVisitor().execute(context);
	simulation::MechanicalAccumulateConstraint(_numConstraints).execute(context);
	_mu = mu.getValue();
	
	// debug
	//std::cout<<" accumulateConstraint_done "  <<std::endl;

	// necessary ///////
	if (_numConstraints > MAX_NUM_CONSTRAINTS)
	{
		serr<<sendl<<"Error in MasterContactSolver, maximum number of contacts exceeded, "<< _numConstraints/3 <<" contacts detected"<<sendl;
		exit(-1);
	}

	lcp->getMu() = _mu;

    _dFree->resize(_numConstraints);
	// as _Wdiag is a sparse matrix resize do not allocate memory
	_Wdiag->resize(_numConstraints,_numConstraints); 
	_result->resize(_numConstraints);

	// debug
	//std::cout<<" resize done "  <<std::endl;
		
	MechanicalGetConstraintValueVisitor(_dFree).execute(context);

    if (this->f_printLog.getValue()) sout<<"MasterContactSolver: "<<_numConstraints<<" constraints, mu = "<<_mu<<sendl;

	//debug
	//std::cout<<" computeCompliance in "  << constraintCorrections.size()<< " constraintCorrections" <<std::endl;

   
	if (initial_guess.getValue())
	{
		MechanicalGetContactIDVisitor(_cont_id_list).execute(context);
		computeInitialGuess();
	}	
		
	

	
	

}



void MasterContactSolver::computeInitialGuess()
{
	int numContact = (_mu > 0.0) ? _numConstraints/3 : _numConstraints;

	for (int c=0; c<numContact; c++)
	{
		if (_mu>0.0){
			(*_result)[3*c  ] = 0.0;
			(*_result)[3*c+1] = 0.0;
			(*_result)[3*c+2] = 0.0;
		}
		else
		{
			(*_result)[c] =  0.0;
			(*_result)[c+numContact] =  0.0;
		}
	}


	for (int c=0; c<numContact; c++)
	{
		for (unsigned int pc=0; pc<_numPreviousContact; pc++)
		{
			if (_cont_id_list[c] == _PreviousContactList[pc].id)
			{
				if (_mu>0.0){
					(*_result)[3*c  ] = _PreviousContactList[pc].F.x();
					(*_result)[3*c+1] = _PreviousContactList[pc].F.y();
					(*_result)[3*c+2] = _PreviousContactList[pc].F.z();
				}
				else
				{
					(*_result)[c+numContact] =  _PreviousContactList[pc].F.x();
				}
			}
		}
	}
}

void MasterContactSolver::keepContactForcesValue()
{
	_numPreviousContact=0;

	int numContact = (_mu > 0.0) ? _numConstraints/3 : _numConstraints;

	for (int c=0; c<numContact; c++)
	{
		if (_mu>0.0)
		{
			if ((*_result)[3*c]>0.0)//((_result[3*c]>0.0)||(_result[3*c+1]>0.0)||(_result[3*c+2]>0.0))
			{
				_PreviousContactList[_numPreviousContact].id = (_cont_id_list[c] >= 0) ? _cont_id_list[c] : -_cont_id_list[c];
				_PreviousContactList[_numPreviousContact].F.x() = (*_result)[3*c];
				_PreviousContactList[_numPreviousContact].F.y() = (*_result)[3*c+1];
				_PreviousContactList[_numPreviousContact].F.z() = (*_result)[3*c+2];
				_numPreviousContact++;
			}
		}
		else
		{
			if ((*_result)[c]>0.0)
			{
				_PreviousContactList[_numPreviousContact].id = (_cont_id_list[c] >= 0) ? _cont_id_list[c] : -_cont_id_list[c];
				_PreviousContactList[_numPreviousContact].F.x() = (*_result)[c];
				_numPreviousContact++;
			}

		}
	}
}

void MasterContactSolver::step(double dt)
{

	context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
	CTime *timer;
	CTime *timerTotal;
	double time = 0.0;
	double timeTotal=0.0;
	double timeScale = 1000.0 / (double)CTime::getRefTicksPerSec();
	if ( displayTime.getValue() )
	{
		timer = new CTime();
		timerTotal = new CTime();
		time = (double) timer->getTime();
		timeTotal = (double) timerTotal->getTime();
		sout<<"********* Start Iteration : " << _numConstraints << " constraints *********" <<sendl;
	}

	// Update the BehaviorModels
	// Required to allow the RayPickInteractor interaction
	
	if( f_printLog.getValue())
		serr<<"updatePos called"<<sendl;

	simulation::BehaviorUpdatePositionVisitor updatePos(dt);
	context->execute(&updatePos);
	
	if( f_printLog.getValue())
		serr<<"updatePos performed - beginVisitor called"<<sendl;


    simulation::MechanicalBeginIntegrationVisitor beginVisitor(dt);
    context->execute(&beginVisitor);

	if( f_printLog.getValue())
		serr<<"beginVisitor performed - SolveVisitor for freeMotion is called"<<sendl;	
	
	// Free Motion
    simulation::SolveVisitor freeMotion(dt, true);
    context->execute(&freeMotion);
	simulation::MechanicalPropagateFreePositionVisitor().execute(context);
	
	if( f_printLog.getValue())
		serr<<" SolveVisitor for freeMotion performed - propagate DX called"<<sendl;	

	core::componentmodel::behavior::BaseMechanicalState::VecId dx_id = core::componentmodel::behavior::BaseMechanicalState::VecId::dx();
	simulation::MechanicalVOpVisitor(dx_id).execute( context);
	simulation::MechanicalPropagateDxVisitor(dx_id).execute( context);
	simulation::MechanicalVOpVisitor(dx_id).execute( context);

	if ( displayTime.getValue() )
	{
		sout << " >>>>> Begin display MasterContactSolver time" << sendl;
		sout<<" Free Motion " << ( (double) timer->getTime() - time)*timeScale <<" ms" <<sendl;

		time = (double) timer->getTime();
	}
	
	if( f_printLog.getValue())
		serr<<" propagate DXn performed - collision called"<<sendl;		

	// Collision detection and response creation
	computeCollision();

	if ( displayTime.getValue() )
	{
		sout<<" computeCollision " << ( (double) timer->getTime() - time)*timeScale <<" ms" <<sendl;
		time = (double) timer->getTime();
	}
//	MechanicalResetContactForceVisitor().execute(context);

	sout<<"constraintCorrections is called"<<sendl;
	
    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->resetContactForce();
	}
	
	sout<<"constraintCorrections is finished"<<sendl;
	if(build_lcp.getValue())
	{
		sout<<"build_LCP is called"<<sendl;
		build_LCP();
		
		
		if ( displayTime.getValue() )
		{
			sout<<" build_LCP " << ( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;
			time = (double) timer->getTime();
		}
		
		double _tol = tol.getValue();
		int _maxIt = maxIt.getValue();
		
		if (_mu > 0.0)
		{
			
			lcp->setNbConst(_numConstraints);
			lcp->setTol(_tol);
			helper::nlcp_gaussseidel(_numConstraints, _dFree->ptr(), _W->lptr(), _result->ptr(), _mu, _tol, _maxIt, initial_guess.getValue());
			if (this->f_printLog.getValue()) helper::afficheLCP(_dFree->ptr(), _W->lptr(), _result->ptr(),_numConstraints);
		}
		else
		{
			// warning _A has been being suppr... need to be allocated
			//
			//		helper::lcp_lexicolemke(_numConstraints, _dFree->ptr(), _W->lptr(), _A.lptr(), _result->ptr());
			helper::gaussSeidelLCP1(_numConstraints, _dFree->ptr(), _W->lptr(), _result->ptr(), _tol, _maxIt);
			if (this->f_printLog.getValue()) helper::afficheLCP(_dFree->ptr(), _W->lptr(), _result->ptr(),_numConstraints);
		}
	}
	else
	{

		
			
		build_problem_info();
		//std::cout<<"build_problem_info is finished"<<std::endl;
		if ( displayTime.getValue() )
		{
			sout<<" build_problem " << ( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;
			time = (double) timer->getTime();
		}		
		
		//std::cout<<"gaussseidel_unbuilt"<<std::endl;
		//std::cout<<"_result-before :"<<_result<<std::endl;
		gaussseidel_unbuilt(_dFree->ptr(), _result->ptr());
		//std::cout<<"\n_result unbuilt:"<<(*_result)<<std::endl;

	   /////// debug
	  /* 
	   _result->resize(_numConstraints);
	   
	   double _tol = tol.getValue();
	   int _maxIt = maxIt.getValue();
	   
	   build_LCP();
	   helper::nlcp_gaussseidel(_numConstraints, _dFree->ptr(), _W->lptr(), _result->ptr(), _mu, _tol, _maxIt, initial_guess.getValue());
		std::cout<<"\n_result nlcp :"<<(*_result)<<std::endl;
		*/	
	   //std::cout<<"LCP:"<<std::endl;
	   //helper::afficheLCP(_dFree->ptr(), _W->lptr(), _result->ptr(),_numConstraints);
		//std::cout<<"build_problem_info is called"<<std::endl;
		
	   ////////			
		
	
	}

	if ( displayTime.getValue() )
	{
		sout<<" TOTAL solve_LCP" <<( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;
		time = (double) timer->getTime();
	}

	if (initial_guess.getValue())
		keepContactForcesValue();

	if(this->f_printLog.getValue())
	{
		serr<<"keepContactForces done"<<sendl;

	}

//	MechanicalApplyContactForceVisitor(_result).execute(context);
    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->applyContactForce(_result);
	}
	if(this->f_printLog.getValue())
	{
		serr<<"applyContactForce in constraintCorrection done"<<sendl;

	}

	simulation::MechanicalPropagateAndAddDxVisitor().execute( context);

	//simulation::MechanicalPropagatePositionAndVelocityVisitor().execute(context);

	//simulation::MechanicalPropagateAndAddDxVisitor().execute( context);

	if(this->f_printLog.getValue())
	{
		serr<<"propagate corrective motion done"<<sendl;

	}

//	MechanicalResetContactForceVisitor().execute(context);
    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->resetContactForce();
	}
	if ( displayTime.getValue() )
	{
		sout<<" contactCorrections" <<( (double) timer->getTime() - time)*timeScale <<" s" <<sendl;
		sout << "<<<<<< End display MasterContactSolver time." << sendl;
	}


	if(build_lcp.getValue())
	{
		lcp = (lcp == &lcp1) ? &lcp2 : &lcp1;
		_W = &lcp->W;
		_dFree = &lcp->dFree;
		_result = &lcp->f;
	}


	//lcp1.unlock();
	//lcp2.unlock();
	//sout << "new lcp : " << lcp << endl;

	//struct timespec ts;
	//ts.tv_sec = 0;
    //ts.tv_nsec = 60 *1000 *1000;
//	nanosleep(&ts, NULL);

    simulation::MechanicalEndIntegrationVisitor endVisitor(dt);
    context->execute(&endVisitor);


	if (displayTime.getValue())
	{
		sout<<" TotalTime" <<( (double) timerTotal->getTime() - timeTotal)*timeScale <<" ms" <<sendl;
	}

}


int MasterContactSolver::gaussseidel_unbuilt(double *dfree, double *f)
{

	CTime *timer;
	double time = 0.0;
	double timeScale = 1000.0 / (double)CTime::getRefTicksPerSec();
	if ( displayTime.getValue() )
	{
		timer = new CTime();
		time = (double) timer->getTime();
	}


	if(_mu==0.0)
	{	
		serr<<"WARNING: frictionless case with unbuilt lcp is not implemented"<<sendl;
	}
	
	/////// test: numContacts = _numConstraints/3 (must be dividable by 3)
	double test = _numConstraints/3;
	int numContacts =  (int) floor(test);
	test = _numConstraints/3 - numContacts;

	if (test>0.01){
		serr<<" WARNING dim should be dividable by 3 in nlcp_gaussseidel"<<sendl;
		return 0;
	}
	//////////////////////////////////////////////
	// iterators
	int it,c1;
	
	//////////////////////////////////////////////
	// data for iterative procedure
	double _tol = tol.getValue();
	int _maxIt = maxIt.getValue();
	double _mu = mu.getValue();
	
	//debug
	//std::cout<<"data are set"<<std::endl;				
	
	//////// Important component if the LCP is not build : 
	// for each contact, the pair of constraint correction that is involved with the contact is memorized
	_cclist_elem1.clear();
	_cclist_elem2.clear();
	for (c1=0; c1<numContacts; c1++){
		bool elem1 = false;
		bool elem2 = false;	
		for (unsigned int i=0;i<constraintCorrections.size();i++)
		{

			core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
			if(cc->hasConstraintNumber(3*c1))
			{
				if(elem1){
					_cclist_elem2.push_back(cc);
					elem2=true;
				}
				else
				{
					_cclist_elem1.push_back(cc);
					elem1=true;
				}
				
			}
		}
		if(!elem1)
				serr<<"WARNING: no constraintCorrection found for contact"<<c1<<sendl;
		if(!elem2)
				_cclist_elem2.push_back(NULL);
				
	}
	
	//debug
	//std::cout<<"_cclist_elem1 _cclist_elem2 are set"<<std::endl;		
	
		
	
	// memory allocation of vector d
	double *d;
	d = (double*)malloc(_numConstraints*sizeof(double));	
	


	//////////////
	// Beginning of iterative computations
	//////////////
	
	/// each constraintCorrection has an internal force vector that is set to "0"
	
	// if necessary: modify the sequence of contact
	std::list<int> contact_sequence;
	
	for (unsigned int i=0;i<constraintCorrections.size();i++)
	{
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];	
		cc->resetForUnbuiltResolution(f, contact_sequence); 
	}
	
	bool change_contact_sequence = false;
	
	if(contact_sequence.size() ==_numConstraints)
		change_contact_sequence=true;
	
	
	
	/////////// the 3x3 diagonal block matrix is built:
	/////////// for each contact, the pair of constraintcorrection is called to add the contribution
	for (c1=0; c1<numContacts; c1++)
	{
		//debug
		//std::cout<<"contact "<<c1<<" cclist_elem1 : "<<_cclist_elem1[c1]->getName()<<std::endl;
		// compliance of object1
		_cclist_elem1[c1]->getBlockDiagonalCompliance(_Wdiag, 3*c1, 3*c1+2);			
		// compliance of object2 (if object2 exists)
		if(_cclist_elem2[c1] != NULL){
			_cclist_elem2[c1]->getBlockDiagonalCompliance(_Wdiag, 3*c1, 3*c1+2);
			// debug
			//std::cout<<"_cclist_elem2[c1]"<<std::endl;
		}
	}		
	
					

	// allocation of the inverted system 3x3 
	// TODO: evaluate the cost of this step : it can be avoied by directly feeding W33 in constraint correction 
	helper::LocalBlock33 **W33;
	W33 = (helper::LocalBlock33 **) malloc (_numConstraints*sizeof(helper::LocalBlock33));
	for (c1=0; c1<numContacts; c1++){
		W33[c1] = new helper::LocalBlock33();
		double w[6];
		w[0] = _Wdiag->element(3*c1  , 3*c1  );
		w[1] = _Wdiag->element(3*c1  , 3*c1+1);
		w[2] = _Wdiag->element(3*c1  , 3*c1+2);
		w[3] = _Wdiag->element(3*c1+1, 3*c1+1);
		w[4] = _Wdiag->element(3*c1+1, 3*c1+2);
		w[5] = _Wdiag->element(3*c1+2, 3*c1+2);
		W33[c1]->compute(w[0], w[1] , w[2], w[3], w[4] , w[5]);
	}			
	 
	// debug
	// std::cout<<"getBlockDiagonalCompliance  Wdiag = "<<(* _Wdiag)<<std::endl;
	// return 1;
	if ( displayTime.getValue() )
	{
		sout<<" build_constraints_and_diagonal " << ( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;
		time = (double) timer->getTime();
	}
	
	double error = 0;
	double dn, dt, ds, fn, ft, fs;

	for (it=0; it<_maxIt; it++)
	{
		std::list<int>::iterator it_c = contact_sequence.begin();
		error =0;
		for (int c=0; c<numContacts; c++)
		{
			if(change_contact_sequence)
			{
				int constraint = *it_c;
				c1 = constraint/3;
				it_c++; it_c++; it_c++;
				
			}
			else
				c1=c;
		
			//std::cout<<"it"<<it << " - c1 :"<<c1<<std::endl;
			
			// compute the current violation :
			
			// violation when no contact force
			d[3*c1]=dfree[3*c1]; d[3*c1+1]=dfree[3*c1+1]; d[3*c1+2]=dfree[3*c1+2];
			
			// debug		
			//if(c1<2)
			//	std::cout<<"free displacement for contact : dn_free = "<< d[3*c1] <<"  dt_free = "<<d[3*c1+1]<<"  ds_free = "<<d[3*c1+2]<<std::endl;
			
			
			// displacement of object1 due to contact force
			_cclist_elem1[c1]->addConstraintDisplacement(d, 3*c1, 3*c1+2);
			
			// displacement of object2 due to contact force (if object2 exists)
			if(_cclist_elem2[c1] != NULL)
				_cclist_elem2[c1]->addConstraintDisplacement(d, 3*c1, 3*c1+2);
			
				
		// set current force in fn, ft, fs
			fn=f[3*c1]; ft=f[3*c1+1]; fs=f[3*c1+2];
			
			// set displacement in dn, dt, ds
			dn=d[3*c1]; dt=d[3*c1+1]; ds=d[3*c1+2];												
			
			
			// debug
			//if(c1<2)
			//	std::cout<<"New_GS_State called : dn = "<<dn<<"  dt = "<<dt<<"  ds = "<<ds<<"  fn = "<<fn<<"  ft = "<<ft<<"  fs = "<<fs<<std::endl;
		
		
		

		
		// compute a new state for stick/slip 		
			/// ATTENTION  NOUVEAU GS_STATE : maintenant dn, dt et ds inclue les forces fn, ft, fs
			W33[c1]->New_GS_State(_mu,dn,dt,ds,fn,ft,fs);
			// debug
			//if(c1<2)
			//	std::cout<<"New_GS_State solved for contact "<<c1<<" : dn = "<<dn<<"  dt = "<<dt<<"  ds = "<<ds<<"  fn = "<<fn<<"  ft = "<<ft<<"  fs = "<<fs<<std::endl;
			
		// evaluate an error (based on displacement)
			error += helper::absError(dn,dt,ds,d[3*c1],d[3*c1+1],d[3*c1+2]);
			
			
			
			
			bool update;
			if (f[3*c1] == 0.0 && fn == 0.0)
				update=false;
			else
				update=true;

		// set the new force :
			// compute the Delta of contact forces:
			f[3*c1  ] = fn - f[3*c1  ];
			f[3*c1+1] = ft - f[3*c1+1];
			f[3*c1+2] = fs - f[3*c1+2];
			
			//std::cout<<"fn = "<< fn<<" -  ft = "<< ft<<" -  fs = "<<fs<<std::endl;
			///////// verifier si Delta force vaut 0 => pas la peine d'ajouter la force
			
			// set Delta force on object 1 for evaluating the followings displacement

			if(update)
			{
			_cclist_elem1[c1]->setConstraintDForce(f, 3*c1, 3*c1+2, update);  

			// set Delta force on object2 (if object2 exists)
			if(_cclist_elem2[c1] != NULL)
				_cclist_elem2[c1]->setConstraintDForce(f, 3*c1, 3*c1+2, update);
			}	
			
			
			///// debug : verifie si on retrouve le même dn
			/*
			d[3*c1]=dfree[3*c1]; d[3*c1+1]=dfree[3*c1+1]; d[3*c1+2]=dfree[3*c1+2];
			_cclist_elem1[c1]->addConstraintDisplacement(d, 3*c1, 3*c1+2);
			if(fabs(dn-d[3*c1]) > 0.000000001*fabs(dn) && dn> 0.1*_tol)
				std::cerr<<"WARNING debug : dn ="<<dn<<" d["<<3*c1<<"]= "<< d[3*c1]<<" dfree= "<<dfree[3*c1]<<"  - update :"<<update<<" with fn ="<<fn<<" and f["<<3*c1<<"]= "<< fn-f[3*c1  ]<<std::endl;
			*/
						
			// set force on the contact force vector
			helper::set3Dof(f,c1,fn,ft,fs);	

		}

		if (error < _tol*(numContacts+1)){
			free(d);
			if ( displayTime.getValue() )
			{
				sout<<"convergence after "<<it<<" iterations - error"<<error<<sendl;
			}
			//debug
			//std::cout<<" f : ["<<std::endl;
			for (int i = 0; i < numContacts; i++){
			//	std::cout<<f[3*i]<<"\n"<<f[3*i+1] <<"\n"<<f[3*i+2] <<std::endl;
				delete W33[i];
			}
			//std::cout<<"];"<<std::endl;
			free(W33);
			if ( displayTime.getValue() )
			{
				sout<<" GAUSS_SEIDEL iterations  " << ( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;

			}			
			
			
			return 1;
		}
	}
	free(d);
	for (int i = 0; i < numContacts; i++)
		delete W33[i];
	free(W33);
	if ( displayTime.getValue() )
	{
		sout<<" GAUSS_SEIDEL iterations  " << ( (double) timer->getTime() - time)*timeScale<<" ms" <<sendl;
	}
	
	std::cerr<<"\n No convergence in  unbuilt gaussseidel function : error ="<<error <<" after"<< it<<" iterations"<<std::endl;
	//afficheLCP(dfree,W,f,dim);
	return 0;
	
				


}


SOFA_DECL_CLASS(MasterContactSolver)

int MasterContactSolverClass = core::RegisterObject("Constraint solver")
.add< MasterContactSolver >()
;

} // namespace odesolver

} // namespace component

} // namespace sofa
