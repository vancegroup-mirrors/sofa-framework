// Author: Francois Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#ifndef EULERSTABILIZEDSOLVER_H
#define EULERSTABILIZEDSOLVER_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <Sofa-old/Components/CGImplicitSolver.h>

namespace Sofa
{
namespace Core
{
class MultiVector;
}

namespace Components
{

using Core::MultiVector;

class EulerStabilizedSolver : public CGImplicitSolver
{
public:

    EulerStabilizedSolver();
    virtual void init();
    virtual ~EulerStabilizedSolver();

    /// Compute one iteration step, the call the energy control method
    void solve (double dt);

protected:
    /// Control the mechanical energy, during the integration step.
    double stepLengthToDesiredEnergyAlongDx();
    /// Compute the increase of mechanical energy associated with a given step length along the current search direction, with respect to m_desiredEnergy.
    double mechanicalEnergyIncrease( double step );
    /// Find the step length which sets the enrgy to the desired value
    double stepLengthToDesiredEnergyInInterval( double x1, double fa, double x2, double fb, double tol);


protected:

private:
    MultiVector* m_dx;
    MultiVector* m_positionAtStart;
    MultiVector* m_velocityAtStart;
    double m_dt;
    double m_desiredEnergy;

};

} // namespace Components

} // namespace Sofa


#endif
