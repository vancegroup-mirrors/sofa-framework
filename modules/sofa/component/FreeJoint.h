// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#ifndef SOFA_COMPONENT_FREEJOINT_H
#define SOFA_COMPONENT_FREEJOINT_H

#include <sofa/component/Joint.h>

namespace sofa
{

namespace component
{

/**
	Joint with one rotational DOF along axis z.
 
	@author François Faure
 */

class FreeJoint : public Joint
{

public:
        typedef ArticulatedBody::SolidTypes SolidTypes;
        typedef SolidTypes::Real Real;
        typedef SolidTypes::Vec Vec;
        typedef SolidTypes::Rot Rot;
        typedef SolidTypes::Transform Frame;
        typedef SolidTypes::SpatialVector SpatialVector;
        typedef Frame Coord;
        typedef SpatialVector Deriv;

        FreeJoint( ArticulatedBody* parent, ArticulatedBody* child );
	/// Set the initial velocity
	FreeJoint* setInitialVelocity( const SpatialVector& );
	/// Set the initial displacement
        FreeJoint* setInitialPosition( const Frame& );
	/// Reset angle and velocity to their initial values
	virtual void resetValues();

        //virtual void apply(Frame& , const Frame&);
/*        virtual void applyJ(SpatialVector&, const SpatialVector&);
        virtual void applyJT(SpatialVector& parentForce, const SpatialVector& childForce);*/
        virtual void accumulateForce();

        // get values in parent frame
/*        virtual Vec getLinearVelocity() const;
        virtual Vec getAngularVelocity() const;*/
        
        virtual void propagateX();
        virtual SpatialVector getSpatialAcceleration();


        virtual void vOp(VecId v, VecId a = VecId::null(), VecId b = VecId::null(), double f=1.0);
        virtual double vDot(VecId a, VecId b);
        virtual void doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt);

        // Convert internal dof values to spatial values
        virtual Frame getJointPosition();
        virtual SpatialVector getJointVelocity();
//         virtual SpatialVector getJointForce();
//         virtual SpatialVector getJointDisplacement();

        virtual Frame getFrame(unsigned);
        virtual SpatialVector getSpatialVector(unsigned);
        
        virtual void setX(VecId v);
        virtual void setV(VecId v);
        virtual void setF(VecId v);
        virtual void setDx(VecId v);
    protected:
        helper::vector<Coord*> coords;
        helper::vector<Deriv*> derivs;
        Coord* x_;
        Deriv* v_;
        Deriv* f_;
        Deriv* dx_;
        Coord initialX_;
        Deriv initialV_;

	/// Set the displacement
        FreeJoint* setPosition( const Frame& );
        
        
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

        //Frame sp_Si;  sp_Si is the identity matrix, we remove it from the equations.
        ArticulatedInertia sp_Hi;  ///< auxiliary value
        defaulttype::Mat<6,6,Real> sp_Di_inv; ///< auxiliary value
        SpatialVector sp_Ci;  ///< auxiliary value

        
        ArticulatedInertia getHandleInertia();
        SpatialVector getHandleBias();

};

} // namespace component

} // namespace sofa

#endif
