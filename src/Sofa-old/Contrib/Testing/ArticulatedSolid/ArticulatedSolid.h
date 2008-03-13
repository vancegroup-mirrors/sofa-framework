//
// C++ Interface: ArticulatedSolid
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef Sofa_CoreArticulatedSolid_h
#define Sofa_CoreArticulatedSolid_h

#include <string>
//#include <Sofa-old/Core/DynamicModel.h>
#include <Sofa-old/Components/RigidObject.h>
//#include <Sofa-old/Abstract/VisualModel.h>
#include <Sofa-old/Components/Common/Factory.h>
#include <Sofa-old/Abstract/BehaviorModel.h>
#include "ArticulatedSolidTypes.h"
#include <Sofa-old/Components/GL/Repere.h>

namespace Sofa
{

namespace Components
{
using namespace Sofa::Core::Encoding;

/**
Featherstone's Articulated Body
 
@author François Faure
*/
template<class T>
	//class ArticulatedSolid : public Sofa::Core::DynamicModel, public Sofa::Abstract::VisualModel
	class ArticulatedSolid : public Sofa::Components::RigidObject
	{
public:
        typedef typename T::Transform Transform;
        typedef typename T::SpatialVector SpatialVector;
        typedef typename T::Vec Vec;
	typedef typename T::Rot Rot;
	typedef typename T::Mat Mat;
	typedef typename T::Real Real;
	typedef typename T::RigidInertia RigidInertia;
	typedef typename T::ArticulatedInertia ArticulatedInertia;

        ArticulatedSolid(std::string s="");

        ~ArticulatedSolid();
        
	/// Define the joint frame with respect to the parent
	void setIntraLinkFrame( const Vec& c, const Vec& axis, Real angle );
	void setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx );

	
        // -- VisualModel interface
        void draw();
        void initTextures()
        { }
        void update()
        { }


        Transform& worldTransform()
        {
                return _worldTransform;
        }
        const Transform& worldTransform() const
        {
                return _worldTransform;
        }

        SpatialVector& worldVelocity()
        {
                return sp_Vi;
        }
        const SpatialVector& worldVelocity() const
        {
                return sp_Vi;
        }

        void addChild( ArticulatedSolid* );

protected:
        ArticulatedSolid* _parent;
	std::vector<ArticulatedSolid*> _children;
        Transform _worldTransform;
        Transform _intraLinkTransform;
        Transform _jointTransform;
	SpatialVector sp_Vi; ///< spatial velocity in world coordinates
	SpatialVector sp_Ai; ///< spatial acceleration in world coordinates
	SpatialVector sp_Pi;  ///< bias force
	SpatialVector sp_Ci; ///< auxiliary value
	RigidInertia sp_I; ///< Rigid body inertia matrix in local coordinates
	ArticulatedInertia sp_Ia; ///< Articulated body inertia matrix in world coordinates
	
    public:
	const SpatialVector& get_sp_Ai(){ return sp_Ai; }
	virtual ArticulatedInertia getHandleInertia() const=0;
	virtual SpatialVector getHandleBias(unsigned) const=0;

  
  public:
        // -- DynamicModel interface
    virtual void applyTranslation(double /*dx*/, double /*dy*/, double /*dz*/)
    { }
    virtual void applyScale(double /*sx*/, double /*sy*/, double /*sz*/, double /*smass*/)
    { }
    virtual void applyRotation(double /*ax*/, double /*ay*/, double /*az*/, double /*angle*/)
    { }
    
    typedef Transform VecCoord;
        typedef SpatialVector VecDeriv;

protected:
        double lastResult; ///< Last v_dot result


public:

	virtual void init();

        virtual void beginIteration(double /*dt*/){}

        virtual void endIteration(double /*dt*/){}

        //virtual void resetForce(unsigned result);

        virtual void computeForce(unsigned result);

        virtual void propagateDx(unsigned dx);

        virtual void computeDf(unsigned df);

        virtual void applyConstraints(unsigned dx);

         virtual void integrateVelocity(unsigned res, unsigned x, unsigned v, Real dt)=0;
// 
         virtual void propagatePositionAndVelocity(unsigned x, unsigned v)=0;

         virtual void addMDx(unsigned df, unsigned dx);
         virtual void accFromF(unsigned a, unsigned f)=0;

        /// Wait for the completion of previous operations and return the result of the last v_dot call
        virtual double finish();

        /// Start the execution of the specified operation
        virtual void execute(Opcode operation, VecId res, VecId a, VecId b, double f);

        //virtual void resize(int size);

         virtual void setObject(Abstract::BehaviorModel* obj);
// 
        virtual void compute_vderiv_dot_vderiv(unsigned a, unsigned b)=0;
        virtual void clearVecCoord(unsigned a)=0;
        virtual void clearVecDeriv(unsigned a)=0;
        virtual void vcoord_eq_vcoord(unsigned a, unsigned b)=0;
        virtual void vderiv_eq_vderiv(unsigned a, unsigned b)=0;
        virtual void vderiv_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f)=0;
        virtual void vcoord_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f)=0;
        virtual void vderiv_teq_scalar(unsigned a, Real f)=0;

        

}
;

typedef ArticulatedSolid< ArticulatedSolidTypes<float> > ArticulatedSolidf;

}

}

#endif

