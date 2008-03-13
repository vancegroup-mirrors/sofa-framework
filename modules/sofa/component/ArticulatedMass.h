// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#ifndef SOFA_COMPONENT_ARTICULATEDMASS_H
#define SOFA_COMPONENT_ARTICULATEDMASS_H

#include <sofa/core/componentmodel/behavior/Mass.h>
#include <sofa/defaulttype/SolidTypes.h>

namespace sofa
{

namespace component
{

class ArticulatedBody;

class ArticulatedMass : public core::componentmodel::behavior::Mass< defaulttype::SolidTypes<float> >
{
public:
	typedef core::objectmodel::BaseContext::SolidTypes SolidTypes;
	typedef SolidTypes::Real Real;
	typedef SolidTypes::Vec Vec;
	typedef SolidTypes::RigidInertia RigidInertia;
	typedef SolidTypes::ArticulatedInertia ArticulatedInertia;
	
	ArticulatedMass(ArticulatedBody*);
	virtual ~ArticulatedMass()
	{}
	
	void setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx );
	
	virtual void addMDx(VecDeriv& f, const VecDeriv& dx); ///< f += M dx
	
	virtual void accFromF(VecDeriv& a, const VecDeriv& f); ///< dx = M^-1 f
	
	virtual void computeForce(VecDeriv f, const VecCoord& x, const VecDeriv& v); /// f += gravity and inertia forces
	
	virtual void computeDf(VecDeriv df, const VecCoord& x, const VecDeriv& v, const VecDeriv& dx);
	
	RigidInertia sp_I; ///< Rigid body inertia matrix in local coordinates
	ArticulatedInertia sp_Ia; ///< Articulated body inertia matrix in world coordinates

protected:
	ArticulatedBody* body_;

};

} // namespace component

} // namespace sofa

#endif
