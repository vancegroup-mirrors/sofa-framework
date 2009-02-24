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
#ifndef SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_MASTERCONTACTSOLVER_H

#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/common/MasterSolverImpl.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/componentmodel/behavior/BaseConstraintCorrection.h>
#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/common/OdeSolverImpl.h>
#include <sofa/component/linearsolver/FullMatrix.h>

namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using namespace sofa::component::linearsolver;
using namespace helper::system::thread;

class LCP
{
public:
	int maxConst;
	LPtrFullMatrix<double> W, A;
    FullVector<double> dFree, f;
	double tol;
	int numItMax;
	unsigned int nbConst;
	bool useInitialF;
	double mu;
	int dim;
private:
	bool lok;

public:
	LCP(unsigned int maxConstraint);
	~LCP();
	void reset(void);
	//LCP& operator=(LCP& lcp);
	inline double** getW(void) {return W.lptr();};
	inline double& getMu(void) { return mu;};
	inline double* getDfree(void) {return dFree.ptr();};
	inline int getDfreeSize(void) {return dFree.size();};
	inline double getTolerance(void) {return tol;};
	inline void setTol(double t) {tol = t;};
	inline double getMaxIter(void) {return numItMax;};
	inline double* getF(void) {return f.ptr();};
	inline bool useInitialGuess(void) {return useInitialF;};
	inline unsigned int getNbConst(void) {return nbConst;};
	inline void setNbConst(unsigned int nbC) {nbConst = nbC;};
	inline unsigned int getMaxConst(void) {return maxConst;};

	inline bool isLocked(void) {return false;};
	inline void lock(void) {lok = true;};
	inline void unlock(void) {lok = false;};
	inline void wait(void) {while(lok) ; } //infinite loop?
};



class MechanicalResetContactForceVisitor : public simulation::MechanicalVisitor
{
public:
    VecId force;
	MechanicalResetContactForceVisitor()
	{
	}

	virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->resetContactForce();
		return RESULT_CONTINUE;
	}

	virtual Result fwdMappedMechanicalState(simulation::Node* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->resetForce();
		return RESULT_CONTINUE;
	}
};

/* ACTION 2 : Apply the Contact Forces on mechanical models & Compute displacements */
class MechanicalApplyContactForceVisitor : public simulation::MechanicalVisitor
{
public:
    VecId force;
	MechanicalApplyContactForceVisitor(double *f):_f(f)
	{
	}
	virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
	{
		ms->applyContactForce(_f);
		return RESULT_CONTINUE;
	}

	virtual Result fwdMappedMechanicalState(simulation::Node* /*node*/, core::componentmodel::behavior::BaseMechanicalState* ms)
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
class MechanicalGetConstraintValueVisitor : public simulation::MechanicalVisitor
{
  public:

	MechanicalGetConstraintValueVisitor(double *v): _v(v) // , _numContacts(numContacts)
	{
	}

    virtual Result fwdConstraint(simulation::Node* /*node*/, core::componentmodel::behavior::BaseConstraint* c)
	{
            //std::cout << c->getName()<<"->getConstraintValue()"<<std::endl;
		c->getConstraintValue(_v /*, _numContacts*/);
		return RESULT_CONTINUE;
	}
private:
	double* _v; // vector for constraint values
	// unsigned int &_numContacts; // we need an offset to fill the vector _v if differents contact class are created
};


class MechanicalGetContactIDVisitor : public simulation::MechanicalVisitor
{
public:
	MechanicalGetContactIDVisitor(long *id, unsigned int offset = 0)
		: _id(id),_offset(offset)
	{
	}

    virtual Result fwdConstraint(simulation::Node* /*node*/, core::componentmodel::behavior::BaseConstraint* c)
	{
		c->getConstraintId(_id, _offset);
		return RESULT_CONTINUE;
	}

private:
	long *_id;
	unsigned int _offset;
};



class MasterContactSolver : public sofa::simulation::MasterSolverImpl//, public sofa::simulation::OdeSolverImpl
{
public:
	Data<bool> initial_guess;
	Data < double > tol;
	Data < int > maxIt;
	Data < double > mu;

	MasterContactSolver();
    // virtual const char* getTypeName() const { return "MasterSolver"; }

    void step (double dt);

	//virtual void propagatePositionAndVelocity(double t, VecId x, VecId v);

	virtual void init();
	LCP* getLCP(void) {return (lcp == &lcp1) ? &lcp2 : &lcp1;};

private:
	std::vector<core::componentmodel::behavior::BaseConstraintCorrection*> constraintCorrections;
	void computeInitialGuess();
	void keepContactForcesValue();

	void build_LCP();

	unsigned int _numConstraints;
	double _mu;
	simulation::tree::GNode *context;
	LCP lcp1, lcp2;
	LPtrFullMatrix<double> *_A, *_W;
	FullVector<double> *_dFree, *_result;
	LCP* lcp;

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
