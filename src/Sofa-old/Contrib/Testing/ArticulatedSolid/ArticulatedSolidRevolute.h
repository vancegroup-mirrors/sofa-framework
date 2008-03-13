//
// C++ Interface: ArticulatedSolidRevolute
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef Sofa_ComponentsArticulatedSolidRevolute_h
#define Sofa_ComponentsArticulatedSolidRevolute_h

#include <ArticulatedSolidEulerXYZ.h>
#include <Sofa-old/Components/Common/vector.h>
using Sofa::Components::Common::vector;

namespace Sofa
{

namespace Components
{

/**
ArticulatedSolid with a revolute parent joint.
 
@author The SOFA team
*/
template<class T>
class ArticulatedSolidRevolute : public Sofa::Components::ArticulatedSolidEulerXYZ<T>
{
public:
  typedef typename T::Real Real;
  typedef typename T::Vec Vec;
  typedef typename T::SpatialVector SpatialVector;
  typedef typename T::ArticulatedInertia ArticulatedInertia;

        ArticulatedSolidRevolute();

        ~ArticulatedSolidRevolute();

        void propagatePositionAndVelocity(unsigned x, unsigned v);
        void integrateVelocity( unsigned res, unsigned a, unsigned b, Real dt );
	void accFromF(unsigned a, unsigned f);
	void computeForce(unsigned result);
	
/*	void addHandleInertiaAndBias( ArticulatedInertia& inertia, SpatialVector& bias);*/
	ArticulatedInertia getHandleInertia() const;
	SpatialVector getHandleBias(unsigned) const;

protected:
        vector<Real> angle;
        vector<Real> angularVelocity;
	SpatialVector sp_Si;  ///< dof spatial axis in world coordinates
	SpatialVector sp_Hi;  ///< auxiliary value
	Real sp_Di; ///< auxiliary value
	//Real sp_Ui; ///< auxiliary value

};

}

}

#endif

