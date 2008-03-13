// Author: Francois Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include "EulerStabilizedSolver.h"
#include "Sofa-old/Core/MultiVector.h"
#include "Sofa-old/Components/Common/ObjectFactory.h"
#include "Sofa-old/Components/Common/rmath.h"
#include "Sofa-old/Components/Graph/MechanicalComputeEnergyAction.h"

#include <math.h>
#include <iostream>
using std::cerr;
using std::endl;
#include <limits>
using std::numeric_limits;
#include <assert.h>

using namespace Sofa::Components::Graph;

namespace Sofa
{

namespace Components
{

inline void printerror( const char* msg )
{
    std::cerr<<msg<<std::endl;
    assert(0);
}

using namespace Common;
using namespace Core;

EulerStabilizedSolver::EulerStabilizedSolver()
{
}

void EulerStabilizedSolver::init()
{
    m_dx = new MultiVector( this, V_DERIV );
    m_velocityAtStart = new MultiVector( this, V_DERIV );
    m_positionAtStart = new MultiVector( this, V_COORD );
}

EulerStabilizedSolver::~EulerStabilizedSolver()
{
    delete m_dx;
    delete m_positionAtStart;
    delete m_velocityAtStart;
}



void EulerStabilizedSolver::solve(double dt)
{
    MultiVector pos(this, VecId::position());
    MultiVector vel(this, VecId::velocity());
    m_dt = dt;

    // record current state 
    m_positionAtStart->eq(pos);
    m_velocityAtStart->eq(vel);
    
    if( printLog() )
    {
        cerr<<"EulerStabilizedSolver, dt = "<< dt <<endl;
        cerr<<"EulerStabilizedSolver, initial x = "<< pos <<endl;
        cerr<<"EulerStabilizedSolver, initial v = "<< vel <<endl;
    }

    // compute the displacement during dt
    computeAcc ( getTime(), *m_dx, pos, vel);
    m_dx->teq( dt );
    m_dx->peq( vel );
    m_dx->teq( dt );
    
    // find a point with desired energy in the given direction
    m_desiredEnergy = 0.5;
    double step = stepLengthToDesiredEnergyAlongDx();
    //cerr<<"EulerStabilizedSolver::solve, step = "<<step<<endl;
    
    // update state
    pos.eq(*m_positionAtStart);
    pos.peq(*m_dx,step);
    vel.eq(*m_dx);
    vel.teq(step/dt);
    
    if( printLog() )
    {
        cerr<<"EulerStabilizedSolver, x after step = "<< pos <<endl;
        cerr<<"EulerStabilizedSolver, v after step = "<< vel <<endl;
    }

}

double EulerStabilizedSolver::stepLengthToDesiredEnergyAlongDx()
{
    // A null step cancels the velocity. If the current velocity is not null, this corresponds to a decrease of energy. 
    double stepForEnergyDecrease = 0;
    double energyDecrease = mechanicalEnergyIncrease( stepForEnergyDecrease );
/*    cerr<<"EulerStabilizedSolver::stepLengthToDesiredEnergyAlongDx, energyDecrease = "<<energyDecrease<<" for a step of "<<stepForEnergyDecrease<<endl;*/
    
    //Find a step which results in energy increase
    double stepForEnergyIncrease = 1.0;
    double energyIncrease = mechanicalEnergyIncrease( stepForEnergyIncrease );
/*    cerr<<"EulerStabilizedSolver::stepLengthToDesiredEnergyAlongDx, energyIncrease = "<<energyIncrease<<" for a step of "<<stepForEnergyIncrease<<endl;*/
    while( energyIncrease < 0 ){
        stepForEnergyIncrease *= 2;
        energyIncrease = mechanicalEnergyIncrease( stepForEnergyIncrease );
/*        cerr<<"EulerStabilizedSolver::stepLengthToDesiredEnergyAlongDx, energyIncrease = "<<energyIncrease<<" for a step of "<<stepForEnergyIncrease<<endl;*/
    }
/*    cerr<<"EulerStabilizedSolver::stepLengthToDesiredEnergyAlongDx, stepForEnergyIncrease = "<<stepForEnergyIncrease<<endl;*/
    
    // We now have bracketed the desired step.
    // Find the step with constant energy in the given interval using one-dimensional root finding
    double stepPrecision = 1.0e-4;
    return stepLengthToDesiredEnergyInInterval( stepForEnergyDecrease, energyDecrease, stepForEnergyIncrease, energyIncrease, stepPrecision );
}

double EulerStabilizedSolver::mechanicalEnergyIncrease( double step )
{
    MultiVector pos( this, VecId::position());
    MultiVector vel( this, VecId::velocity());
    MultiVector a( this, V_DERIV );
    double t = getContext()->getTime() + m_dt;
        
    // Set state according to step length
    pos.eq(*m_positionAtStart);
    pos.peq(*m_dx,step);
    vel.eq(*m_dx);
    vel.teq(step/m_dt);
    
    // Compute the mechanical energy in this new state
    MechanicalPropagatePositionAndVelocityAction(t,pos,vel).execute( getContext() );
    MechanicalComputeForceAction(a).execute( getContext() );
    MechanicalComputeEnergyAction computeEnergy;
    computeEnergy.execute( getContext() );
    
/*    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, step = "<<step<<endl;
    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, pos = "<<pos<<endl;
    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, vel = "<<vel<<endl;
    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, kinetic energy = "<< computeEnergy.getKineticEnergy() <<endl;
    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, potential energy = "<< computeEnergy.getPotentialEnergy() <<endl;
    cerr<<"EulerStabilizedSolver::mechanicalEnergyIncrease, desired energy = "<< m_desiredEnergy <<endl;*/
    
    
    return computeEnergy.getKineticEnergy() + computeEnergy.getPotentialEnergy() - m_desiredEnergy;    
}

double EulerStabilizedSolver::stepLengthToDesiredEnergyInInterval( double x1, double fa, double x2, double fb, double tol)
{
/*    cerr<<"EulerStabilizedSolver::stepLengthToDesiredEnergyInInterval start"<<endl;*/
    const int ITMAX=100;
    const double EPS=numeric_limits<double>::epsilon();
    int iter;
    double a=x1,b=x2,c=x2,d=0,e=0,min1,min2;
    double fc,p,q,r,s,tol1,xm;

    if ((fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0))
        printerror("Root must be bracketed in stepLengthToDesiredEnergyInInterval");
    fc=fb;
    for (iter=0;iter<ITMAX;iter++) {
        if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)) {
            c=a;
            fc=fa;
            e=d=b-a;
        }
        if (fabs(fc) < fabs(fb)) {
            a=b;
            b=c;
            c=a;
            fa=fb;
            fb=fc;
            fc=fa;
        }
        tol1=2.0*EPS*fabs(b)+0.5*tol;
        xm=0.5*(c-b);
        if (fabs(xm) <= tol1 || fb == 0.0) { 
            //cerr<<"stepLengthToDesiredEnergyInInterval:1, fabs(xm) = "<<fabs(xm)<<", fb = "<<fb<<endl; 
            return b; 
        }
        if (fabs(e) >= tol1 && fabs(fa) > fabs(fb)) {
            s=fb/fa;
            if (a == c) {
                p=2.0*xm*s;
                q=1.0-s;
            } else {
                q=fa/fc;
                r=fb/fc;
                p=s*(2.0*xm*q*(q-r)-(b-a)*(r-1.0));
                q=(q-1.0)*(r-1.0)*(s-1.0);
            }
            if (p > 0.0) q = -q;
            p=fabs(p);
            min1=3.0*xm*q-fabs(tol1*q);
            min2=fabs(e*q);
            if (2.0*p < (min1 < min2 ? min1 : min2)) {
                e=d;
                d=p/q;
            } else {
                d=xm;
                e=d;
            }
        } else {
            d=xm;
            e=d;
        }
        a=b;
        fa=fb;
        if (fabs(d) > tol1)
            b += d;
        else
            b += SIGN(tol1,xm);
        fb=mechanicalEnergyIncrease(b);
    }
    printerror("Maximum number of iterations exceeded in stepLengthToDesiredEnergyInInterval");
    return 0.0;
}



void create(EulerStabilizedSolver*& obj, ObjectDescription* arg)
{
    obj = new EulerStabilizedSolver();
    obj->parseFields( arg->getAttributeMap() );
}

SOFA_DECL_CLASS(EulerStabilizedSolver)

Creator<ObjectFactory, EulerStabilizedSolver> EulerStabilizedSolverClass("EulerStabilized");



} // namespace Components

} // namespace Sofa




