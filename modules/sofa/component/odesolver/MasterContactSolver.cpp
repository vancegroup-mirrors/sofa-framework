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

#define MAX_NUM_CONSTRAINTS 600

MasterContactSolver::MasterContactSolver()
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
	simulation::tree::MechanicalComputeComplianceVisitor(_W).execute(context);
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

	// Collision detection and response creation
	computeCollision();

	MechanicalResetContactForceVisitor().execute(context);

	build_LCP();

	if (_mu > 0.0)
	{
		double tol = 0.0001;
		int maxIt = 10000;
		helper::nlcp_gaussseidel(_numConstraints, _dFree, _W, _result, _mu, tol, maxIt);
	}
	else
	{
		double tol = 0.0001;
		int maxIt = 10000;
//		helper::lcp_lexicolemke(_numConstraints, _dFree, _W, _A, _result);
		helper::gaussSeidelLCP1(_numConstraints, _dFree, _W, _result, tol, maxIt);
	}

	MechanicalApplyContactForceVisitor(_result).execute(context);

	// Constrained Motion
	for(simulation::tree::GNode::ChildIterator it = context->child.begin(); it != context->child.end(); ++it) 
	{
		for ( unsigned i=0; i<(*it)->solver.size(); i++)
		{
			(*it)->solver[i]->solve(dt);
		}
	}

	MechanicalResetContactForceVisitor().execute(context);

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
