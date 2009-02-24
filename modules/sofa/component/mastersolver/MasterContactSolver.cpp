/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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

using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

namespace odesolver
{


LCP::LCP(unsigned int mxC) : maxConst(mxC), tol(0.00001), numItMax(1000), useInitialF(true), mu(0.0), dim(0), lok(false)
{
	W.resize(maxConst,maxConst);
    A.resize(maxConst,2*maxConst+1);
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
:initial_guess(initData(&initial_guess, true, "initial_guess","activate LCP results history to improve its resolution performances."))
,tol( initData(&tol, 0.001, "tolerance", ""))
,maxIt( initData(&maxIt, 1000, "maxIt", ""))
,mu( initData(&mu, 0.6, "mu", ""))
,_mu(0.6)
, lcp1(MAX_NUM_CONSTRAINTS)
, lcp2(MAX_NUM_CONSTRAINTS)
, _A(&lcp1.A)
, _W(&lcp1.W)
, _dFree(&lcp1.dFree)
, _result(&lcp1.f)
, lcp(&lcp1)
{
	_numConstraints = 0;
	_mu = 0.0;

	_numPreviousContact=0;
	_PreviousContactList = (contactBuf *)malloc(MAX_NUM_CONSTRAINTS * sizeof(contactBuf));
	_cont_id_list = (long *)malloc(MAX_NUM_CONSTRAINTS * sizeof(long));
}

void MasterContactSolver::init()
{
    getContext()->get<core::componentmodel::behavior::BaseConstraintCorrection>(&constraintCorrections, core::objectmodel::BaseContext::SearchDown);
}

void MasterContactSolver::build_LCP()
{
	_numConstraints = 0;
//std::cout<<" accumulateConstraint "  <<std::endl;

	// mechanical action executed from root node to propagate the constraints
	simulation::MechanicalResetConstraintVisitor().execute(context);
	_mu = mu.getValue();
	simulation::MechanicalAccumulateConstraint(_numConstraints, _mu).execute(context);
	_mu = mu.getValue();


	//std::cout<<" accumulateConstraint_done "  <<std::endl;

	if (_numConstraints > MAX_NUM_CONSTRAINTS)
	{
		cerr<<endl<<"Error in MasterContactSolver, maximum number of contacts exceeded, "<< _numConstraints/3 <<" contacts detected"<<endl;
		exit(-1);
	}

	lcp->getMu() = _mu;

    _dFree->resize(_numConstraints);
    _W->resize(_numConstraints,_numConstraints);

	MechanicalGetConstraintValueVisitor(_dFree->ptr()).execute(context);
//	simulation::MechanicalComputeComplianceVisitor(_W).execute(context);

    if (this->f_printLog.getValue()) std::cout<<"MasterContactSolver: "<<_numConstraints<<" constraints, mu = "<<_mu<<std::endl;

	//std::cout<<" computeCompliance in "  << constraintCorrections.size()<< " constraintCorrections" <<std::endl;

    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->getCompliance(_W);
	}
	//std::cout<<" computeCompliance_done "  <<std::endl;

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

	context = dynamic_cast<simulation::tree::GNode *>(this->getContext()); // access to current node

#ifdef DISPLAY_TIME
	CTime *timer = new CTime();
	double time = 0;
	time = (double) timer->getTime();
#endif

	// Update the BehaviorModels
	// Required to allow the RayPickInteractor interaction

	simulation::BehaviorUpdatePositionVisitor updatePos(dt);
	context->execute(&updatePos);


    simulation::MechanicalBeginIntegrationVisitor beginVisitor(dt);
    context->execute(&beginVisitor);

	// Free Motion
    simulation::SolveVisitor freeMotion(dt);
    context->execute(&freeMotion);
	simulation::MechanicalPropagateFreePositionVisitor().execute(context);

	core::componentmodel::behavior::BaseMechanicalState::VecId dx_id = core::componentmodel::behavior::BaseMechanicalState::VecId::dx();
	simulation::MechanicalVOpVisitor(dx_id).execute( context);
	simulation::MechanicalPropagateDxVisitor(dx_id).execute( context);
	simulation::MechanicalVOpVisitor(dx_id).execute( context);

#ifdef DISPLAY_TIME
	std::cout<<" Free Motion " << ( (double) timer->getTime() - time)*0.001 <<" ms" <<std::endl;
	time = (double) timer->getTime();
#endif

	// Collision detection and response creation
	computeCollision();

#ifdef DISPLAY_TIME
	std::cout<<" computeCollision " << ( (double) timer->getTime() - time)*0.001 <<" ms" <<std::endl;
	time = (double) timer->getTime();
#endif
//	MechanicalResetContactForceVisitor().execute(context);

    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->resetContactForce();
	}

	build_LCP();

#ifdef DISPLAY_TIME
	std::cout<<" build_LCP " << ( (double) timer->getTime() - time)*0.001<<" ms" <<std::endl;
	time = (double) timer->getTime();
#endif

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

//		helper::lcp_lexicolemke(_numConstraints, _dFree->ptr(), _W->lptr(), _A.lptr(), _result->ptr());
		helper::gaussSeidelLCP1(_numConstraints, _dFree->ptr(), _W->lptr(), _result->ptr(), _tol, _maxIt);
		if (this->f_printLog.getValue()) helper::afficheLCP(_dFree->ptr(), _W->lptr(), _result->ptr(),_numConstraints);
	}

#ifdef DISPLAY_TIME
	std::cout<<" solve_LCP" <<( (double) timer->getTime() - time)*0.001<<" ms" <<std::endl;
	time = (double) timer->getTime();
#endif

	if (initial_guess.getValue())
		keepContactForcesValue();


//	MechanicalApplyContactForceVisitor(_result).execute(context);
    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->applyContactForce(_result);
	}

	simulation::MechanicalPropagateAndAddDxVisitor().execute( context);
	simulation::MechanicalPropagatePositionAndVelocityVisitor().execute(context);


//	MechanicalResetContactForceVisitor().execute(context);
    for (unsigned int i=0;i<constraintCorrections.size();i++)
    {
		core::componentmodel::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
		cc->resetContactForce();
	}
#ifdef DISPLAY_TIME
	std::cout<<" contactCorrections" <<( (double) timer->getTime() - time)*0.001 <<" ms" <<std::endl;
#endif


	//switch lcp
	//cout << "switch lcp : " << lcp << endl;

	//lcp1.wait();
	//lcp1.lock();
	//lcp2.wait();
	//lcp2.lock();

	lcp = (lcp == &lcp1) ? &lcp2 : &lcp1;
	_A =&lcp->A;
	_W = &lcp->W;
	_dFree = &lcp->dFree;
	_result = &lcp->f;


	//lcp1.unlock();
	//lcp2.unlock();
	//cout << "new lcp : " << lcp << endl;

	//struct timespec ts;
	//ts.tv_sec = 0;
    //ts.tv_nsec = 60 *1000 *1000;
//	nanosleep(&ts, NULL);

    simulation::MechanicalEndIntegrationVisitor endVisitor(dt);
    context->execute(&endVisitor);
}

SOFA_DECL_CLASS(MasterContactSolver)

int MasterContactSolverClass = core::RegisterObject("Constraint solver")
.add< MasterContactSolver >()
;

} // namespace odesolver

} // namespace component

} // namespace sofa
