// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#ifndef SOFA_COMPONENT_JOINT_H
#define SOFA_COMPONENT_JOINT_H

#include <sofa/component/ArticulatedBody.h>
#include <sofa/core/componentmodel/behavior/MechanicalMapping.h>
#include <sofa/core/componentmodel/behavior/Constraint.h>
#include <sofa/core/objectmodel/ContextObject.h>

namespace sofa
{

namespace component
{

class ArticulatedMass;
    
/**
	Joint between two articulated bodies. 
        Current implementation allows to apply Featherstone's linear-time articulated solid dynamics using reduced coordinates. This is a pure abstract class. The derived classes implement internal DOFs which model the reduced coordinates of the body. The type of these internal DOFs depend on the type of Joint (number and kind of DOFs).
        Inherits from MechanicalMapping because if transmits motion top-down and forces bottom-up 
        Inherits from Constraint because it could be processed as a constraint between 6-DOF solids. However, this is not yet implemented.
 
	@author François Faure
 */
class Joint:
	public core::componentmodel::behavior::MechanicalMapping<ArticulatedBody,ArticulatedBody>,
	public core::componentmodel::behavior::Constraint<ArticulatedBody>
{
        typedef core::componentmodel::behavior::MechanicalMapping<ArticulatedBody,ArticulatedBody> Mapping;
public:
        typedef ArticulatedBody::SolidTypes SolidTypes;
	typedef SolidTypes::Real Real;
	typedef SolidTypes::Vec Vec;
	typedef SolidTypes::Rot Rot;
	typedef SolidTypes::Transform Frame;
	typedef SolidTypes::SpatialVector SpatialVector;
        typedef SolidTypes::RigidInertia RigidInertia;
        typedef SolidTypes::ArticulatedInertia ArticulatedInertia;
        typedef core::componentmodel::behavior::BaseMechanicalState::VecId VecId;

        Joint( ArticulatedBody* parent, ArticulatedBody* child );
        virtual ~Joint()
        {}
	
	ArticulatedBody* getParent();
	ArticulatedBody* getChild();
	Joint* setMassNode( ArticulatedMass* );

        // Constraint interface
        virtual void projectResponse(VecDeriv&){};
        virtual void projectVelocity(VecDeriv&){}
        virtual void projectPosition(VecCoord&){}
        virtual core::componentmodel::behavior::BaseMechanicalState* getDOFs();
        
        
        /// Define the joint frame with respect to the parent
        Joint* setGeometry( const Vec& c, const Rot& ori );


	virtual void init();
	virtual void propagateX()=0;
	
	
        void setIntraLinkFrame(const Frame&);
        Frame& getIntraLinkFrame();
        const Frame& getIntraLinkFrame() const;
	
	//
	
	// get values in parent frame 
	Frame getPosition();
	SpatialVector getVelocity();
	
	Frame getCoord(unsigned);
	SpatialVector getDeriv(unsigned);
	

        void addToIa( const ArticulatedInertia& );
        void addToBias( const SpatialVector& );
        virtual void resetValues()=0;
        virtual SpatialVector getSpatialAcceleration()=0;
        virtual void vOp(VecId v, VecId a = VecId::null(), VecId b = VecId::null(), double f=1.0)=0;
        virtual double vDot(VecId a, VecId b)=0;
        virtual void doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt)=0;
        virtual Frame getFrame(unsigned)=0;
        virtual SpatialVector getSpatialVector(unsigned)=0;
        
        // DOF-related methods
        virtual void setX(VecId v)=0; 
        virtual void setV(VecId v)=0;
        virtual void setF(VecId v)=0;
        virtual void setDx(VecId v)=0;
    protected:
       
        Frame m_intraLinkTransform;
        Frame m_jointTransform;
        Frame jointFrame_;
	ArticulatedMass* massNode_;
	SpatialVector sp_Vi;  ///< dof spatial axis in world coordinates
        ArticulatedInertia sp_Ia;
        SpatialVector sp_Pi;
        
        Joint* getJointOfParent();
	
	
        // Convert internal dof values to spatial values, in joint coordinates
        virtual Frame getJointPosition()=0;
        virtual SpatialVector getJointVelocity()=0;
    private:
        Joint* m_jointOfParent;
        
        // currently unused
        virtual void apply(Frame& , const Frame&){}
        virtual void applyJ(SpatialVector&, const SpatialVector&){}
        virtual void applyJT(SpatialVector&, const SpatialVector&){}
};


} // namespace component

} // namespace sofa

#endif
