//
// C++ Interface: ArticulatedBody
//
// Description:
//
//
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_COMPONENT_ARTICULATEDBODY_H
#define SOFA_COMPONENT_ARTICULATEDBODY_H

#include <sofa/component/contextobject/GuidedCoordinateSystem.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/componentmodel/behavior/Mass.h>
// #include <sofa/core/Mapping.h>

namespace sofa
{

namespace component
{

class Joint;
// class ArticulatedMass;

/**
Featherstone's Articulated Body. WARNING: work in progress
 
@author François Faure
*/
class ArticulatedBody :
	public core::componentmodel::behavior::MechanicalState<core::objectmodel::BaseContext::SolidTypes>,
	public contextobject::CoordinateSystem,
	public core::componentmodel::behavior::Mass<core::objectmodel::BaseContext::SolidTypes>
{
public:
    typedef core::objectmodel::BaseContext::SolidTypes SolidTypes;
    typedef SolidTypes::Vec Vec;
    typedef SolidTypes::Rot Rot;
    typedef SolidTypes::Mat Mat;
    typedef SolidTypes::Real Real;
    typedef SolidTypes::Transform Frame;
    typedef Frame Coord;
    typedef Frame VecCoord;
    typedef SolidTypes::SpatialVector SpatialVector;
    typedef SpatialVector Deriv;
    typedef SpatialVector VecDeriv;
    typedef SolidTypes::RigidInertia RigidInertia;
    typedef SolidTypes::ArticulatedInertia ArticulatedInertia;

	// Constraints Interface
	typedef SolidTypes::VecConst VecConst;

    ArticulatedBody();

    ~ArticulatedBody();

    virtual void init();
    virtual void reset();
    /// Set the parent the joint
    void setJoint( Joint* );
    /// parent joint
    Joint* getJoint();
    /// parent ArticulatedBody
    ArticulatedBody* getParent();
    virtual void apply(); // update context
    
    /// Set transform with respect to parent, in parent coordinates
    virtual void setTransform( const Frame& f );
    /// Set velocity with respect to parent, in parent coordinates
    void setVelocity( const SpatialVector& v );
    /// Set velocity with respect to parent, in parent coordinates
    void setLinearVelocity( const Vec& v );
    /// Set velocity with respect to parent, in parent coordinates
    void setAngularVelocity( const Vec& v );
    /// Velocity with respect to parent, in parent coordinates
    const SpatialVector& getVelocity() const;
    /// Position of the local frame wrt world
    const Frame& getPositionInWorld() const;
    /// Velocity wrt world, in world coordinates
    const SpatialVector& getVelocityInWorld() const;

    /**@name methods:visual
    Methods related to the VisualModel
     */
    ///@{
    virtual void draw();
    virtual void initTextures()
    {}
    virtual void update()
    {}
    ///@}



    /// @name interface of MechanicalState
    /// @{

    virtual void setX(VecId v);
    virtual void setV(VecId v);
    virtual void setF(VecId v);
    virtual void setDx(VecId v);
	virtual void setC(VecId /*c*/){};

    Frame* getX();
    const Frame* getX() const;
    const Frame* getX0() const;

    SpatialVector* getV();
    const SpatialVector* getV() const;
    const SpatialVector* getV0() const;

    SpatialVector* getF();
    const SpatialVector* getF() const;

    SpatialVector* getDx();
    const SpatialVector* getDx() const;

	VecConst* getC(){return c;};
	const VecConst* getC() const {return c;};

    virtual void vOp(VecId v, VecId a = VecId::null(), VecId b = VecId::null(), double f=1.0);
    virtual double vDot(VecId a, VecId b);
    virtual void doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt);

    void resize(int)
    {}

    virtual void beginIteration(double /*dt*/)
    {}

    virtual void endIteration(double /*dt*/)
    {}

    virtual void resetForce();

    virtual void accumulateForce()
    {}

    virtual void accumulateDf()
    {}

	virtual void resetConstraint(){};

    void vAlloc(VecId )
    {}

    void vFree(VecId )
    {}

    virtual void getIndicesInSpace(sofa::helper::vector<unsigned>& /*indices*/, Real /*xmin*/, Real /*xmax*/,Real /*ymin*/, Real /*ymax*/, Real /*zmin*/, Real /*zmax*/) const;

    /// @}


    /**@name interface of Mass
    */
    ///@{

public:
    void setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx );
    const RigidInertia& getInertia() const;

    virtual void addMDx(VecDeriv& f, const VecDeriv& dx); ///< f += M dx

    virtual void accFromF(VecDeriv& a, const VecDeriv& f); ///< dx = M^-1 f

    virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v); /// f += gravity and inertia forces

    virtual void addDForce(VecDeriv& df, const VecCoord& x, const VecDeriv& v, const VecDeriv& dx);
    
    virtual double getKineticEnergy( const VecDeriv& v );
    
    virtual double getPotentialEnergy(const VecCoord& x);


    ///@}




    /**@name methods:inertia
    Methods related to mass and inertia
    */
    ///@{


    Real getMass() const;

    const SpatialVector& get_sp_Ai() const
    {
        return *getDx();
    }
    SpatialVector& get_sp_Ai()
    {
        return *getDx();
    }

    ///@}


    
    virtual std::string getTemplateName() const
    {
        return SolidTypes::Name();
    }

    
    static const char* Name()
    {
        return "ArticulatedBody";
    }


protected:


    /**@name values:inertia
    Values related to mass and inertia
     */
    ///@{
    RigidInertia sp_I; ///< Rigid body inertia matrix in local coordinates
    //ArticulatedInertia sp_Ia; ///< Articulated body inertia matrix in world coordinates
    ///@}


    SpatialVector sp_Ci; ///< auxiliary value


    //	SpatialVector sp_Vi; ///< spatial velocity in world coordinates
    //	SpatialVector sp_Ai; ///< spatial acceleration in world coordinates
    //	SpatialVector sp_Pi;  ///< bias force
    //         virtual void compute_vderiv_dot_vderiv(unsigned a, unsigned b)=0;
    //         virtual void clearVecCoord(unsigned a)=0;
    //         virtual void clearVecDeriv(unsigned a)=0;
    //         virtual void vcoord_eq_vcoord(unsigned a, unsigned b)=0;
    //         virtual void vderiv_eq_vderiv(unsigned a, unsigned b)=0;
    //         virtual void vderiv_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f)=0;
    //         virtual void vcoord_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f)=0;
    //         virtual void vderiv_teq_scalar(unsigned a, Real f)=0;








    /// @name Debug
    /// @{
    virtual void printDOF( VecId, std::ostream& =std::cerr );
    /// @}

    // interface of BasicMechanicalState
    //=================================



protected:
    Joint* joint_;
    ArticulatedBody* parentBody_;
    Frame* getCoord(unsigned);
    SpatialVector* getDeriv(unsigned);

private:
    helper::vector<Frame*> coords_;
    helper::vector<SpatialVector*> derivs_;
    Frame* x_;
    SpatialVector* v_;
    SpatialVector* f_;
    SpatialVector* dx_;

    Frame x0_;
    SpatialVector v0_;

    VecConst *c;
}
;

} // namespace component

} // namespace sofa


#endif


