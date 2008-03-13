// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/FreeJoint.h>
#include <sofa/core/componentmodel/behavior/Mass.h>
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

FreeJoint::FreeJoint( ArticulatedBody* parent, ArticulatedBody* child )
                : Joint( parent, child )
{
        setX(VecId::position());
        setV(VecId::velocity());
        setF(VecId::force());
        setDx(VecId::dx());
        //getX()[0] =  0;
        initialX_ = Frame::identity();
        initialV_ = SpatialVector( Vec(0,0,0), Vec(0,0,0) );
}

FreeJoint* FreeJoint::setInitialVelocity( const SpatialVector& v )
{
    getV() = v;
    initialV_ = v;
    return this;
}

FreeJoint* FreeJoint::setInitialPosition( const Frame& p )
{
    initialX_ = p;
    return setPosition(p);
}
FreeJoint* FreeJoint::setPosition( const Frame& p )
{
    setIntraLinkFrame( Frame( p.getOrigin(), Rot::identity() ) );
    getX() = Frame( Vec(0,0,0), p.getOrientation() );
    cerr<<"FreeJoint::setPosition, new intraLinkTransform = "<<getIntraLinkFrame()<<endl;
    cerr<<"FreeJoint::setPosition, new X = "<<getX()<<endl;
    return this;
}

void FreeJoint::resetValues()
{
    getV() = initialV_;
    setPosition(initialX_);
/*    cerr<<"FreeJoint, reset x to "<<*getX()<<endl;
    cerr<<"FreeJoint, reset v to "<<*getV()<<endl;*/
    Joint::reset();
}

FreeJoint::Coord* FreeJoint::getCoord( unsigned index )
{
        if (index>=coords.size())
                coords.resize(index+1);
        if (coords[index]==NULL)
                coords[index] = new Coord;
        return coords[index];
}

const FreeJoint::Coord* FreeJoint::getCoord( unsigned index ) const
{
        assert(index<coords.size());
        return coords[index];
}


FreeJoint::Deriv* FreeJoint::getDeriv( unsigned index )
{
        if (index>=derivs.size())
                derivs.resize(index+1);
        if (derivs[index]==NULL)
                derivs[index] = new Deriv;
        return derivs[index];
}

const FreeJoint::Deriv* FreeJoint::getDeriv( unsigned index ) const
{
        assert(index<derivs.size());
        return derivs[index];
}

void FreeJoint::setX(VecId i)
{
        x_=getCoord(i.index);
}
void FreeJoint::setV(VecId i)
{
        v_=getDeriv(i.index);
}
void FreeJoint::setF(VecId i)
{
        f_=getDeriv(i.index);
}
void FreeJoint::setDx(VecId i)
{
        dx_=getDeriv(i.index);
}

const FreeJoint::Coord& FreeJoint::getX() const
{
        return *x_;
}
FreeJoint::Coord& FreeJoint::getX()
{
        return *x_;
}
const FreeJoint::Deriv& FreeJoint::getV() const
{
        return *v_;
}
FreeJoint::Deriv& FreeJoint::getV()
{
        return *v_;
}
const FreeJoint::Deriv& FreeJoint::getF() const
{
        return *f_;
}
FreeJoint::Deriv& FreeJoint::getF()
{
        return *f_;
}
const FreeJoint::Deriv& FreeJoint::getDx() const
{
        return *dx_;
}
FreeJoint::Deriv& FreeJoint::getDx()
{
        return *dx_;
}


void FreeJoint::doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt)
{
    assert( !xfrom.isNull() && xfrom.type==VecId::V_COORD );
    assert( !vfrom.isNull() && vfrom.type==VecId::V_DERIV );
    assert( !xto.isNull() && xto.type==VecId::V_COORD );
    assert( !vto.isNull() && vto.type==VecId::V_DERIV );
    assert( !acc.isNull() && acc.type==VecId::V_DERIV );
    
    const Coord& vxfrom = *getCoord(xfrom.index);
    Coord& vxto = *getCoord(xto.index);
    const Deriv& vvfrom = *getDeriv(vfrom.index);
    Deriv& vvto = *getDeriv(vto.index);
    const Deriv& vacc = *getDeriv(acc.index);
    
    
    cerr<<"FreeJoint::doEulerStep, vxfrom = "<< vxfrom <<endl;
    vxto = Frame( vvfrom.getLinearVelocity()*dt, Rot::set( (vvfrom.getAngularVelocity()*dt))*vxfrom.getOrientation()  );
    cerr<<"FreeJoint::doEulerStep, vxto = "<< vxto <<endl;
    vvto = vvfrom;
    
    Frame tmp( vvfrom*dt );
    SpatialVector omega = vvfrom*dt;
    omega.setLinearVelocity( Vec(0,0,0) );
    cerr<<"FreeJoint::doEulerStep, tmp = "<<tmp <<endl;
    cerr<<"FreeJoint::doEulerStep, omega = "<<omega<<endl;
    cerr<<"FreeJoint::doEulerStep, tmp * omega = "<<tmp * omega<<endl;
    SpatialVector increment = vacc*dt;
    increment.setLinearVelocity( increment.getLinearVelocity() -(tmp*omega).getLinearVelocity() );
    cerr<<"FreeJoint::doEulerStep, velocity increment = "<< increment <<endl;
    vvto += increment;
    cerr<<"FreeJoint::doEulerStep, new velocity = "<<vvto<<endl;
    
    if( getCoord(xto.index)==&getX() )
        setPosition( getIntraLinkFrame() * vxto);
    
}

void FreeJoint::vOp(VecId v, VecId a, VecId b, double f)
{
        //cerr<<"    FreeJoint::vOp "<<endl;
    std::cerr << "FreeJoint::vOp ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
    if(v.isNull()) {
                // ERROR
                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                return;
        }
        if (a.isNull()) {
                if (b.isNull()) {
                        // v = 0
                        if (v.type == VecId::V_COORD) {
                                Coord& vv = *getCoord(v.index);
                                vv.clear();
                        } else {
                                Deriv& vv = *getDeriv(v.index);
                                vv.clear();
                        }
                } else {
                        if (b.type != v.type) {
                                // ERROR
                                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                                return;
                        }
                        if (v == b) {
                                // v *= f
                                if (v.type == VecId::V_COORD) {
                                        *getCoord(v.index) *= f;
                                } else {
                                        *getDeriv(v.index) *= f;
                                }
                        } else {
                                // v = b*f
                                if (v.type == VecId::V_COORD) {
                                        Coord& vv = *getCoord(v.index);
                                        Coord& vb = *getCoord(b.index);
                                        vv = vb * f;
                                } else {
                                        Deriv& vv = *getDeriv(v.index);
                                        Deriv& vb = *getDeriv(b.index);
                                        vv = vb * f;
                                }
                        }
                }
        } else {
                if (a.type != v.type) {
                        // ERROR
                        std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                        return;
                }
                if (b.isNull()) { // v = a
                        if (v.type == VecId::V_COORD) {
                                Coord& vv = *getCoord(v.index);
                                Coord& va = *getCoord(a.index);
                                vv = va;
                        } else {
                                Deriv& vv = *getDeriv(v.index);
                                Deriv& va = *getDeriv(a.index);
                                vv = va;
                        }
                } else {
                        if (v == a) {
                                if (f==1.0) {
                                        // v += b
                                        if (v.type == VecId::V_COORD) {
                                                Coord& vv = *getCoord(v.index);
                                                if (b.type == VecId::V_COORD) {
                                                        Coord& vb = *getCoord(b.index);
                                                        vv += vb;
                                                } else {
                                                    Deriv& vb = *getDeriv(b.index);
                                                    cerr << "FreeJoint::vOp pos += vel * 1.0, pos = "<<vv<<", vel = "<<vb<<", h = "<<f<<endl;
                                                    vv += vb;
                                                        cerr << "FreeJoint::vOp pos += vel * h, new pos = "<<vv<<endl;
                                                }
                                        } else if (b.type == VecId::V_DERIV) {
                                                Deriv& vv = *getDeriv(v.index);
                                                Deriv& vb = *getDeriv(b.index);
                                                vv += vb;
                                        } else {
                                                // ERROR
                                                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                                                return;
                                        }
                                } else {
                                        // v += b*f
                                        if (v.type == VecId::V_COORD) {
                                                Coord& vv = *getCoord(v.index);
                                                if (b.type == VecId::V_COORD) {
                                                        Coord& vb = *getCoord(b.index);
                                                        vv += vb*f;
                                                } else {
                                                        Deriv& vb = *getDeriv(b.index);
                                                        cerr << "FreeJoint::vOp pos += vel * h, pos = "<<vv<<", vel = "<<vb<<", h = "<<f<<endl;
                                                        vv += vb*f;
                                                        cerr << "FreeJoint::vOp pos += vel * h, new pos = "<<vv<<endl;
                                                        }
                                        } else if (b.type == VecId::V_DERIV) {
                                                Deriv& vv = *getDeriv(v.index);
                                                Deriv& vb = *getDeriv(b.index);
                                                vv += vb*f;
                                        } else {
                                                // ERROR
                                                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                                                return;
                                        }
                                }
                        } else {
                                if (f==1.0) {
                                        // v = a+b
                                        if (v.type == VecId::V_COORD) {
                                                Coord& vv = *getCoord(v.index);
                                                Coord& va = *getCoord(a.index);
                                                if (b.type == VecId::V_COORD) {
                                                        Coord& vb = *getCoord(b.index);
                                                        vv = va;
                                                        vv += vb;
                                                } else {
                                                        Deriv& vb = *getDeriv(b.index);
                                                        vv = va;
                                                        vv += vb;
                                                }
                                        } else if (b.type == VecId::V_DERIV) {
                                                Deriv& vv = *getDeriv(v.index);
                                                Deriv& va = *getDeriv(a.index);
                                                Deriv& vb = *getDeriv(b.index);
                                                vv = va + vb;
                                        } else {
                                                // ERROR
                                                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                                                return;
                                        }
                                } else {
                                        // v = a+b*f
                                        if (v.type == VecId::V_COORD) {
                                                Coord& vv = *getCoord(v.index);
                                                Coord& va = *getCoord(a.index);
                                                if (b.type == VecId::V_COORD) {
                                                        Coord& vb = *getCoord(b.index);
                                                        vv = va;
                                                        vv += vb*f;
                                                } else {
                                                        Deriv& vb = *getDeriv(b.index);
                                                        vv = va;
                                                        vv += vb*f;
                                                }
                                        } else if (b.type == VecId::V_DERIV) {
                                                Deriv& vv = *getDeriv(v.index);
                                                Deriv& va = *getDeriv(a.index);
                                                Deriv& vb = *getDeriv(b.index);
                                                vv = va + vb*f;
                                        } else {
                                                // ERROR
                                                std::cerr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")\n";
                                                return;
                                        }
                                }
                        }
                }
        }
        if( v.type == VecId::V_COORD && getCoord(v.index)==&getX() ){
            cerr<<"FreeJoint::vOp, previous position is "<< getX() << endl;
            setPosition( getIntraLinkFrame() * getX() );
            cerr<<"FreeJoint::vOp, new position is "<< getX() << endl;
            cerr<<"FreeJoint::vOp, new intraLinkTransform is "<< getIntraLinkFrame() << endl;
        }
}

double FreeJoint::vDot( VecId a, VecId b )
{
        double r = 0.0;
        if (a.type == VecId::V_DERIV && b.type == VecId::V_DERIV) {
                Deriv* va = getDeriv(a.index);
                Deriv* vb = getDeriv(b.index);
                r += (*va) * (*vb);
        } else {
                std::cerr << "Invalid dot operation ("<<a<<','<<b<<")\n";
        }
        return r;
}

FreeJoint::Frame FreeJoint::getJointPosition()
{
        /*    cerr<<"FreeJoint::getJointPosition(),    getX()[0] = "<<getX()[0]<<endl;
            cerr<<"FreeJoint::getJointPosition(),    Frame( axis ) = "<<Frame( axis * getX()[0] )<<endl;
            cerr<<"FreeJoint::getJointPosition(),    joint position = "<<Frame( axis * getX()[0]  )<<endl;*/
        return Frame(  getX() );
}

FreeJoint::SpatialVector FreeJoint::getJointVelocity()
{
        return getV();
}

FreeJoint::Frame FreeJoint::getFrame(unsigned i )
{
        return Frame( (*getCoord(i)) );
}

FreeJoint::SpatialVector FreeJoint::getSpatialVector(unsigned i)
{
        return (*getDeriv(i));
}




void FreeJoint::propagateX() 
{


    cerr<<"FreeJoint::propagateX, start---------------------------------- "<<endl;
        // world transform
    //cerr<<"FreeJoint::propagateX, angle = "<<angle<<endl;
    m_jointTransform = getX();
    Frame solidTransform = m_intraLinkTransform * m_jointTransform;
    Frame worldTransform = getParent() ? getParent()->getPositionInWorld() * solidTransform : solidTransform;
    if( getParent() )
        cerr<<"FreeJoint::propagateX, parent Transform = "<<getParent()->getPositionInWorld()<<endl;
    cerr<<"FreeJoint::propagateX, intraLinkTransform = "<<m_intraLinkTransform<<endl;
    cerr<<"FreeJoint::propagateX, jointTransform = "<<m_jointTransform<<endl;
    cerr<<"FreeJoint::propagateX, worldTransform = "<<worldTransform<<endl;
	// spatial dof
//    sp_Si = worldTransform;
/*    cerr<<"FreeJoint::propagateX, sp_Si = "<<sp_Si<<endl;*/
    // spatial velocity
    if( getParent() ){
/*        cerr<<"FreeJoint::propagateX, parentVelocity = "<<getParent()->getVelocityInWorld()<<endl;*/
        sp_Vi = worldTransform / (getParent()->getVelocityInWorld()) + getV();
    }
    else sp_Vi = getV(); // in local spatial coordinates
    cerr<<"FreeJoint::propagateX, world velocity sp_Vi = "<<sp_Vi<<endl;
    sp_Ia = getChild()->getInertia();// (Keep it in local coordinates) * worldTransform ;
    cerr<<"FreeJoint::propagateX, sp_Ia = "<<sp_Ia<<endl;
	
	// bias force
    sp_Pi = sp_Vi.cross( sp_Ia * sp_Vi );
    cerr<<"FreeJoint::propagateX, sp_Ia * sp_Vi = "<<sp_Ia * sp_Vi<<endl;
    cerr<<"FreeJoint::propagateX, velocity bias force sp_Pi = "<<sp_Pi<<endl;
    
    getChild()->setTransform( solidTransform );  // vraiment utile ?

/*    cerr<<"FreeJoint::propagateX, end---------------------------------- "<<endl;*/
}

void FreeJoint::accumulateForce()
{
/*    cerr<<"FreeJoint::accumulateForce, start---------------------------------- "<<endl;*/
        
	
    sp_Hi = sp_Ia;
/*    cerr<<"FreeJoint::accumulateForce, sp_Hi = "<<sp_Hi<<endl;*/
    defaulttype::Mat<6,6,Real> sp_Di;
    sp_Ia.copyTo(sp_Di);
    defaulttype::luinv( sp_Di_inv, sp_Di );
/*    cerr<<"FreeJoint::accumulateForce, sp_Di = "<<sp_Di<<endl;*/
    sp_Ci = sp_Vi.cross( getV() );
/*    cerr<<"FreeJoint::accumulateForce, sp_Ci = "<<sp_Ci<<endl;*/
    

    SpatialVector& sp_Ui = getF();
    sp_Ui = - (this->sp_Hi * this->sp_Ci + this->sp_Pi);
    cerr<<"FreeJoint::accumulateForce, , sp_Di_inv = "<<sp_Di_inv<<endl;
    cerr<<"FreeJoint::accumulateForce, , sp_Hi = "<<sp_Hi<<endl;
    cerr<<"FreeJoint::accumulateForce, , sp_Ci = "<<sp_Ci<<endl;
    cerr<<"FreeJoint::accumulateForce, , sp_Pi = "<<sp_Pi<<endl;
    cerr<<"FreeJoint::accumulateForce, , sp_Ui = "<<sp_Ui<<endl;
    
    if( getJointOfParent() ){
        getJointOfParent()->addToIa( getHandleInertia() );
        getJointOfParent()->addToBias( getHandleBias() );
    }
    
/*    cerr<<"FreeJoint::accumulateForce, end---------------------------------- "<<endl;*/
}

FreeJoint::ArticulatedInertia FreeJoint::getHandleInertia()
{
    //return sp_Ia - SolidTypes::dyad(sp_Hi,sp_Hi) * (1/sp_Di);
    SolidTypes::Mat m;
    m.fill(0);
    return ArticulatedInertia(m,m,m);
}

FreeJoint::SpatialVector FreeJoint::getHandleBias()
{
    const SpatialVector& sp_Ui = getF();
/*    cerr<<"FreeJoint::getHandleBias, sp_Ui = "<<sp_Ui<<endl;
    cerr<<"FreeJoint::getHandleBias, sp_Di = "<<sp_Di<<endl;
    cerr<<"FreeJoint::getHandleBias, sp_Pi = "<<sp_Pi<<endl;
    cerr<<"FreeJoint::getHandleBias, sp_Ia = "<<sp_Ia<<endl;
    cerr<<"FreeJoint::getHandleBias, sp_Ci = "<<sp_Ci<<endl;
    cerr<<"FreeJoint::getHandleBias, sp_Hi = "<<sp_Hi<<endl;
    cerr<<"FreeJoint::getHandleBias, handle bias = "<< this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * (sp_Ui/sp_Di) <<endl;*/
    return this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * ( sp_Ui * sp_Di_inv);
}

FreeJoint::SpatialVector FreeJoint::getSpatialAcceleration()
{
    SpatialVector& ui = getF();
    SpatialVector& angularAcceleration = getDx();
    SpatialVector parent_sp_Ai;
    if( getParent()!= NULL ){
        parent_sp_Ai = getParent()->get_sp_Ai();
    }
    else {
        parent_sp_Ai =  SpatialVector( Vec(0,0,0), -getContext()->getGravityInWorld() );
    }
    
    angularAcceleration =  (ui - sp_Hi * parent_sp_Ai) * sp_Di_inv;
    cerr<<"FreeJoint::getSpatialAcceleration, ui = "<<ui<<endl;
    cerr<<"FreeJoint::getSpatialAcceleration, sp_Hi = "<<sp_Hi<<endl;
    cerr<<"FreeJoint::getSpatialAcceleration, sp_Hi * parent_sp_Ai = "<<sp_Hi * parent_sp_Ai <<endl;
    cerr<<"FreeJoint::getSpatialAcceleration, sp_Ci = "<<sp_Ci <<endl;
    cerr<<"FreeJoint::getSpatialAcceleration, local acceleration = "<<angularAcceleration<<endl;
    return this->sp_Ci + angularAcceleration;
/*    angularAcceleration = SpatialVector( Vec(0,0,0), Vec(0,0,0) ); // false, just for debug
    return SpatialVector( Vec(0,0,0), Vec(0,0,0) ); // false, just for debug*/
}

}// namespace component
}// namespace sofa


