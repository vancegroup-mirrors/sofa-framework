#include <sofa/component/odesolver/MasterContactSolver.h>

#include <sofa/simulation/tree/AnimateVisitor.h>
#include <sofa/simulation/tree/MechanicalVisitor.h>

#include <sofa/helper/LCPcalc.h>

#include <sofa/core/ObjectFactory.h>

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

using namespace sofa::defaulttype;
using namespace helper::system::thread;
using namespace core::componentmodel::behavior;

#define MAX_NUM_CONSTRAINTS 3000

MasterContactSolver::MasterContactSolver()
:initial_guess(initData(&initial_guess, true, "initial_guess","activate LCP results history to improve its resolution performances."))
#ifdef SOFA_GPU_CUDA
,useGPU(initData(&useGPU, true, "useGPU", "compute LCP using GPU"))
#endif

{
	_W = (double **) malloc(MAX_NUM_CONSTRAINTS * sizeof(double*));
	_A = (double **) malloc(MAX_NUM_CONSTRAINTS * sizeof(double*));
	for (int i=0; i<MAX_NUM_CONSTRAINTS; i++){
		_W[i] = (double*) malloc(MAX_NUM_CONSTRAINTS * sizeof(double));
		_A[i] = (double*) malloc((2 * MAX_NUM_CONSTRAINTS + 1) * sizeof(double));
	}

	_dFree = (double *)malloc(MAX_NUM_CONSTRAINTS * sizeof(double));
	_result= (double *)malloc((2 * MAX_NUM_CONSTRAINTS + 1) * sizeof(double));

	int c1, c2;
	for ( c1=0; c1<MAX_NUM_CONSTRAINTS; c1++)
	{
		_dFree[c1] = 0.0;

		for ( c2=0; c2<MAX_NUM_CONSTRAINTS; c2++)
		{
			_W[c1][c2] = 0.0;
		}
	}

	_numConstraints = 0;
	_mu = 0.0;

	_numPreviousContact=0;
	_PreviousContactList = (contactBuf *)malloc(MAX_NUM_CONSTRAINTS * sizeof(contactBuf));
	_cont_id_list = (long *)malloc(MAX_NUM_CONSTRAINTS * sizeof(long));
}

void MasterContactSolver::init()
{
    getContext()->get<core::componentmodel::collision::BaseContactCorrection>(&contactCorrections, core::objectmodel::BaseContext::SearchDown);
}

void MasterContactSolver::build_LCP()
{
	unsigned int c1, c2;
	for ( c1=0; c1<_numConstraints; c1++)
	{
		_dFree[c1] = 0.0;

		for ( c2=0; c2<_numConstraints; c2++)
		{
			_W[c1][c2] = 0.0;
		}
	}

	_numConstraints = 0;

	// mechanical action executed from root node to propagate the constraints
	simulation::tree::MechanicalResetConstraintVisitor().execute(context);
	simulation::tree::MechanicalAccumulateConstraint(_numConstraints, _mu).execute(context);
	if (_numConstraints > MAX_NUM_CONSTRAINTS)
	{	
		cerr<<endl<<"Error in MasterContactSolver, maximum number of contacts exceeded, "<< _numConstraints/3 <<" contacts detected"<<endl;
		exit(-1);
	}

	MechanicalGetConstraintValueVisitor(_dFree).execute(context);
//	simulation::tree::MechanicalComputeComplianceVisitor(_W).execute(context);

    for (unsigned int i=0;i<contactCorrections.size();i++)
    {
		core::componentmodel::collision::BaseContactCorrection* cc = contactCorrections[i];
		cc->getCompliance(_W);
	}

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
			_result[3*c  ] = 0.0;
			_result[3*c+1] = 0.0;
			_result[3*c+2] = 0.0;
		}
		else
		{
			_result[c] =  0.0;
			_result[c+numContact] =  0.0;
		}
	}


	for (int c=0; c<numContact; c++)
	{
		for (unsigned int pc=0; pc<_numPreviousContact; pc++)
		{
			if (_cont_id_list[c] == _PreviousContactList[pc].id)
			{
				if (_mu>0.0){
					_result[3*c  ] = _PreviousContactList[pc].F.x();
					_result[3*c+1] = _PreviousContactList[pc].F.y();
					_result[3*c+2] = _PreviousContactList[pc].F.z();
				}
				else
				{
					_result[c+numContact] =  _PreviousContactList[pc].F.x();
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
			if (_result[3*c]>0.0)//((_result[3*c]>0.0)||(_result[3*c+1]>0.0)||(_result[3*c+2]>0.0))
			{
				_PreviousContactList[_numPreviousContact].id = (_cont_id_list[c] >= 0) ? _cont_id_list[c] : -_cont_id_list[c];
				_PreviousContactList[_numPreviousContact].F.x() = _result[3*c];
				_PreviousContactList[_numPreviousContact].F.y() = _result[3*c+1];
				_PreviousContactList[_numPreviousContact].F.z() = _result[3*c+2];
				_numPreviousContact++;
			}
		}
		else
		{
			if (_result[c]>0.0)
			{
				_PreviousContactList[_numPreviousContact].id = (_cont_id_list[c] >= 0) ? _cont_id_list[c] : -_cont_id_list[c];
				_PreviousContactList[_numPreviousContact].F.x() = _result[c];
				_numPreviousContact++;
			}

		}
	}
}

void MasterContactSolver::step(double dt)
{
	
	context = dynamic_cast<simulation::tree::GNode *>(this->getContext()); // access to current node

	// Update the BehaviorModels
	// Required to allow the RayPickInteractor interaction
	for(simulation::tree::GNode::ChildIterator it = context->child.begin(); it != context->child.end(); ++it) 
	{
		for ( unsigned i=0; i<(*it)->behaviorModel.size(); i++)
        {
			(*it)->behaviorModel[i]->updatePosition(dt);
        }
	}

    simulation::tree::MechanicalBeginIntegrationVisitor beginVisitor(dt);
    context->execute(&beginVisitor);

	// Free Motion
	for(simulation::tree::GNode::ChildIterator it = context->child.begin(); it != context->child.end(); ++it) 
	{
		for ( unsigned i=0; i<(*it)->solver.size(); i++)
		{
			(*it)->solver[i]->solve(dt);
		}
	}
	simulation::tree::MechanicalPropagateFreePositionVisitor().execute(context);


	// Collision detection and response creation
	computeCollision();

//	MechanicalResetContactForceVisitor().execute(context);

    for (unsigned int i=0;i<contactCorrections.size();i++)
    {
		core::componentmodel::collision::BaseContactCorrection* cc = contactCorrections[i];
		cc->resetContactForce();
	}
	
	build_LCP();

	if (_mu > 0.0)
	{
		double tol = 0.001;
		int maxIt = 10000;

		helper::nlcp_gaussseidel(_numConstraints, _dFree, _W, _result, _mu, tol, maxIt, initial_guess.getValue());
	}
	else
	{
		double tol = 0.001;
		int maxIt = 1000;
//		helper::lcp_lexicolemke(_numConstraints, _dFree, _W, _A, _result);
#ifdef SOFA_GPU_CUDA
		if (useGPU.getValue())
			sofa::gpu::cuda::CudaLCP::CudaGaussSeidelLCP1(_numConstraints, _dFree, _W, _result, tol, maxIt);
		else
#endif
			helper::gaussSeidelLCP1(_numConstraints, _dFree, _W, _result, tol, maxIt);
	}


	if (initial_guess.getValue())
		keepContactForcesValue();


//	MechanicalApplyContactForceVisitor(_result).execute(context);
    for (unsigned int i=0;i<contactCorrections.size();i++)
    {
		core::componentmodel::collision::BaseContactCorrection* cc = contactCorrections[i];
		cc->applyContactForce(_result);
	}

	simulation::tree::MechanicalPropagateAndAddDxVisitor().execute( context);


	/*

	
	// Constrained Motion
	for(simulation::tree::GNode::ChildIterator it = context->child.begin(); it != context->child.end(); ++it) 
	{
		for ( unsigned i=0; i<(*it)->solver.size(); i++)
		{
			(*it)->solver[i]->solve(dt);
		}
	}
	*/



//	MechanicalResetContactForceVisitor().execute(context);
    for (unsigned int i=0;i<contactCorrections.size();i++)
    {
		core::componentmodel::collision::BaseContactCorrection* cc = contactCorrections[i];
		cc->resetContactForce();
	}

    simulation::tree::MechanicalEndIntegrationVisitor endVisitor(dt);
    context->execute(&endVisitor);
}

SOFA_DECL_CLASS(MasterContactSolver)

int MasterContactSolverClass = core::RegisterObject("Constraint solver")
.add< MasterContactSolver >()
;

} // namespace odesolver

} // namespace component

} // namespace sofa
