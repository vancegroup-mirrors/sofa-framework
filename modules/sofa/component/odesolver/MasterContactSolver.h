#ifndef SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H

#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/tree/MasterSolverImpl.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/tree/MechanicalVisitor.h>
#include <sofa/core/componentmodel/collision/BaseContactCorrection.h>
#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/tree/OdeSolverImpl.h>
#ifdef SOFA_GPU_CUDA
#include <sofa/gpu/cuda/CudaLCP.h>
#endif


namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using namespace helper::system::thread;


class MechanicalResetContactForceVisitor : public simulation::tree::MechanicalVisitor
{
public:
    VecId force;
	MechanicalResetContactForceVisitor()
	{
	}

	virtual Result fwdMechanicalState(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->resetContactForce();
		return RESULT_CONTINUE;
	}

	virtual Result fwdMappedMechanicalState(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->resetForce();
		return RESULT_CONTINUE;
	}
};

/* ACTION 2 : Apply the Contact Forces on mechanical models & Compute displacements */
class MechanicalApplyContactForceVisitor : public simulation::tree::MechanicalVisitor
{
public:
    VecId force;
	MechanicalApplyContactForceVisitor(double *f):_f(f)
	{
	}
	virtual Result fwdMechanicalState(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->applyContactForce(_f);
		return RESULT_CONTINUE;
	}

	virtual Result fwdMappedMechanicalState(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->applyContactForce(_f);
		return RESULT_CONTINUE;
	}

private:
	double *_f; // vector of contact forces from lcp //
				// to be multiplied by constraint direction in mechanical models //

};

/* ACTION 3 : gets the vector of constraint values */
/* ACTION 3 : gets the vector of constraint values */
class MechanicalGetConstraintValueVisitor : public simulation::tree::MechanicalVisitor
{
public:
	MechanicalGetConstraintValueVisitor(double *v): _v(v) // , _numContacts(numContacts)
	{
	}

    virtual Result fwdConstraint(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseConstraint* c)
	{
		c->getConstraintValue(_v /*, _numContacts*/);
		return RESULT_CONTINUE;
	}
private:
	double* _v; // vector for constraint values
	// unsigned int &_numContacts; // we need an offset to fill the vector _v if differents contact class are created
};


class MechanicalGetContactIDVisitor : public simulation::tree::MechanicalVisitor
{
public:
	MechanicalGetContactIDVisitor(long *id, unsigned int offset = 0)
		: _id(id),_offset(offset)
	{
	}

    virtual Result fwdConstraint(simulation::tree::GNode* /*node*/, core::componentmodel::behavior::BaseConstraint* c)
	{
		c->getConstraintId(_id, _offset);
		return RESULT_CONTINUE;
	}

private:
	long *_id;
	unsigned int _offset;
};



class MasterContactSolver : public sofa::simulation::tree::MasterSolverImpl//, public sofa::simulation::tree::OdeSolverImpl
{
public:
	Data<bool> initial_guess;
#ifdef SOFA_GPU_CUDA
	Data<bool> useGPU;
#endif

	MasterContactSolver();
    // virtual const char* getTypeName() const { return "MasterSolver"; }
        
    void step (double dt);

	//virtual void propagatePositionAndVelocity(double t, VecId x, VecId v);

	virtual void init();

private:
	std::vector<core::componentmodel::collision::BaseContactCorrection*> contactCorrections;
	void computeInitialGuess();
	void keepContactForcesValue();

	void build_LCP(); 	
	double	**_W, **_A; 
	double	*_dFree, *_result;
	unsigned int _numConstraints;
	double _mu;
	simulation::tree::GNode *context;
	
	typedef struct {
		Vector3 n;
		Vector3 t;
		Vector3 s;
		Vector3 F;
		long id;

	} contactBuf;

	contactBuf *_PreviousContactList;
	unsigned int _numPreviousContact;
	long *_cont_id_list;
};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif
