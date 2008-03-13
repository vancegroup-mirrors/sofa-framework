// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#ifndef SOFA_COMPONENT_REVOLUTEJOINT_H
#define SOFA_COMPONENT_REVOLUTEJOINT_H

#include <sofa/component/Joint.h>

namespace sofa
{

namespace component
{

/**
	Joint with one rotational DOF along axis z.
 
	@author François Faure
 */

class RevoluteJoint : public Joint
{

public:
        typedef ArticulatedBody::SolidTypes SolidTypes;
        typedef SolidTypes::Real Real;
        typedef SolidTypes::Vec Vec;
        typedef SolidTypes::Rot Rot;
        typedef SolidTypes::Transform Frame;
        typedef SolidTypes::SpatialVector SpatialVector;
        typedef defaulttype::Vec<1,Real> Coord;
        typedef defaulttype::Vec<1,Real> Deriv;

        // Specific methods
        
        RevoluteJoint( ArticulatedBody* parent, ArticulatedBody* child );
	/// Set the initial velocity
	RevoluteJoint* setVelocity( Real omega );
	/// Set the initial angle
	RevoluteJoint* setAngle( Real alpha );
	/// Reset angle and velocity to their initial values
	virtual void resetValues();


        
        virtual void propagateX();
        virtual void accumulateForce();
        virtual SpatialVector getSpatialAcceleration();


        // DOF-related methods
        virtual void setX(VecId v);
        virtual void setV(VecId v);
        virtual void setF(VecId v);
        virtual void setDx(VecId v);
        virtual void vOp(VecId v, VecId a = VecId::null(), VecId b = VecId::null(), double f=1.0);
        virtual double vDot(VecId a, VecId b);
        virtual void doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt);

        // Convert internal dof values to spatial values
        virtual Frame getJointPosition();
        virtual SpatialVector getJointVelocity();

        virtual Frame getFrame(unsigned);
        virtual SpatialVector getSpatialVector(unsigned);
protected:
        helper::vector<Coord*> coords;
        helper::vector<Deriv*> derivs;
        Coord* x_;
        Deriv* v_;
        Deriv* f_;
        Deriv* dx_;
        Coord initialAngle_;
        Deriv initialVelocity_;

        Coord* getCoord( unsigned index );
        const Coord* getCoord( unsigned index ) const;
        Deriv* getDeriv( unsigned index );
        const Deriv* getDeriv( unsigned index ) const;

        const Coord& getX() const;
        Coord& getX();
        const Deriv& getV() const;
        Deriv& getV();
        const Deriv& getF() const;
        Deriv& getF();
        const Deriv& getDx() const;
        Deriv& getDx();

        SpatialVector sp_Si;  ///< dof spatial axis in world coordinates
        SpatialVector sp_Hi;  ///< auxiliary value
        Real sp_Di; ///< auxiliary value
        SpatialVector sp_Ci;  ///< auxiliary value
	static SpatialVector axis;  ///< axis of rotation af all RevoluteJoints

        
        ArticulatedInertia getHandleInertia();
        SpatialVector getHandleBias();

};

} // namespace component

} // namespace sofa

#endif
