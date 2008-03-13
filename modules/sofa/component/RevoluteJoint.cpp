// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/RevoluteJoint.h>
#include <sofa/core/componentmodel/behavior/Mass.h>
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

RevoluteJoint::SpatialVector RevoluteJoint::axis ( RevoluteJoint::Vec(1,0,0), RevoluteJoint::Vec(0,0,0) );

RevoluteJoint::RevoluteJoint( ArticulatedBody* parent, ArticulatedBody* child )
                : Joint( parent, child )
{
        setX(VecId::position());
        setV(VecId::velocity());
        setF(VecId::force());
        setDx(VecId::dx());
        //getX()[0] =  0;
	initialAngle_[0] = 0;
	initialVelocity_[0] = 0;
}

RevoluteJoint* RevoluteJoint::setVelocity( Real omega )
{
    getV() = omega;
    initialVelocity_[0] = omega;
    return this;
}

RevoluteJoint* RevoluteJoint::setAngle( Real alpha )
{
    initialAngle_[0] = alpha;
    getX() = alpha;
    return this;
}

void RevoluteJoint::resetValues()
{
    getV() = initialVelocity_[0];
    getX() = initialAngle_[0];
/*    cerr<<"RevoluteJoint, reset x to "<<*getX()<<endl;
    cerr<<"RevoluteJoint, reset v to "<<*getV()<<endl;*/
    Joint::reset();
}

RevoluteJoint::Coord* RevoluteJoint::getCoord( unsigned index )
{
        if (index>=coords.size())
                coords.resize(index+1);
        if (coords[index]==NULL)
                coords[index] = new Coord;
        return coords[index];
}

const RevoluteJoint::Coord* RevoluteJoint::getCoord( unsigned index ) const
{
        assert(index<coords.size());
        return coords[index];
}


RevoluteJoint::Deriv* RevoluteJoint::getDeriv( unsigned index )
{
        if (index>=derivs.size())
                derivs.resize(index+1);
        if (derivs[index]==NULL)
                derivs[index] = new Deriv;
        return derivs[index];
}

const RevoluteJoint::Deriv* RevoluteJoint::getDeriv( unsigned index ) const
{
        assert(index<derivs.size());
        return derivs[index];
}

void RevoluteJoint::setX(VecId i)
{
        x_=getCoord(i.index);
}
void RevoluteJoint::setV(VecId i)
{
        v_=getDeriv(i.index);
}
void RevoluteJoint::setF(VecId i)
{
        f_=getDeriv(i.index);
}
void RevoluteJoint::setDx(VecId i)
{
        dx_=getDeriv(i.index);
}

const RevoluteJoint::Coord& RevoluteJoint::getX() const
{
        return *x_;
}
RevoluteJoint::Coord& RevoluteJoint::getX()
{
        return *x_;
}
const RevoluteJoint::Deriv& RevoluteJoint::getV() const
{
        return *v_;
}
RevoluteJoint::Deriv& RevoluteJoint::getV()
{
        return *v_;
}
const RevoluteJoint::Deriv& RevoluteJoint::getF() const
{
        return *f_;
}
RevoluteJoint::Deriv& RevoluteJoint::getF()
{
        return *f_;
}
const RevoluteJoint::Deriv& RevoluteJoint::getDx() const
{
        return *dx_;
}
RevoluteJoint::Deriv& RevoluteJoint::getDx()
{
        return *dx_;
}


void RevoluteJoint::doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt)
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
    
    vvto = vvfrom;
    vvto += vacc * dt;
    vxto = vxfrom; 
    vxto += vvto * dt;
}
    
    
void RevoluteJoint::vOp(VecId v, VecId a, VecId b, double f)
{
        //cerr<<"    RevoluteJoint::vOp "<<endl;
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
                                                        vv += vb;
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
/*                                                        cerr << "RevoluteJoint::vOp pos += vel * h, pos = "<<vv<<", vel = "<<vb<<", h = "<<f<<endl;*/
                                                        vv += vb*f;
/*                                                        cerr << "RevoluteJoint::vOp pos += vel * h, new pos = "<<vv<<endl;*/
                                                        }
                                        } else if (b.type == VecId::V_DERIV) {
                                                Deriv& vv = *getDeriv(v.index);
                                                Deriv& vb = *getDeriv(b.index);
/*                                                cerr << "RevoluteJoint::vOp vel += acc * h, vel = "<<vv<<", acc = "<<vb<<", h = "<<f<<endl;*/
                                                vv += vb*f;
/*                                                cerr << "RevoluteJoint::vOp vel += acc * h, new vel = "<<vv<<endl;*/
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
}

double RevoluteJoint::vDot( VecId a, VecId b )
{
        double r = 0.0;
        if (a.type == VecId::V_COORD && b.type == VecId::V_COORD) {
                Coord* va = getCoord(a.index);
                Coord* vb = getCoord(b.index);
                r += (*va) * (*vb);
        } else if (a.type == VecId::V_DERIV && b.type == VecId::V_DERIV) {
                Deriv* va = getDeriv(a.index);
                Deriv* vb = getDeriv(b.index);
                r += (*va) * (*vb);
        } else {
                std::cerr << "Invalid dot operation ("<<a<<','<<b<<")\n";
        }
        return r;
}

RevoluteJoint::Frame RevoluteJoint::getJointPosition()
{
        /*    cerr<<"RevoluteJoint::getJointPosition(),    getX()[0] = "<<getX()[0]<<endl;
            cerr<<"RevoluteJoint::getJointPosition(),    Frame( axis ) = "<<Frame( axis * getX()[0] )<<endl;
            cerr<<"RevoluteJoint::getJointPosition(),    joint position = "<<Frame( axis * getX()[0]  )<<endl;*/
        return Frame( axis  * getX()[0] );
}

RevoluteJoint::SpatialVector RevoluteJoint::getJointVelocity()
{
    return axis  * getV()[0];
}


RevoluteJoint::Frame RevoluteJoint::getFrame(unsigned i )
{
        return Frame( axis * (*getCoord(i))[0] );
}

RevoluteJoint::SpatialVector RevoluteJoint::getSpatialVector(unsigned i)
{
        return axis * (*getDeriv(i))[0];
}


void RevoluteJoint::propagateX() 
{

    Real& angle = getX()[0];
    Real& angularVelocity = getV()[0];

/*    cerr<<"RevoluteJoint::propagateX, start---------------------------------- "<<endl;*/
        // world transform
    //cerr<<"RevoluteJoint::propagateX, angle = "<<angle<<endl;
    m_jointTransform = Frame( Rot::createFromRotationVector(Vec(angle,0,0) ), Vec(0,0,0) );
    Frame solidTransform = m_intraLinkTransform * m_jointTransform;
    Frame worldTransform = getParent() ? getParent()->getPositionInWorld() * solidTransform : solidTransform;
/*    if( getParent() )
        cerr<<"RevoluteJoint::propagateX, parent Transform = "<<getParent()->getPositionInWorld()<<endl;
    cerr<<"RevoluteJoint::propagateX, intraLinkTransform = "<<m_intraLinkTransform<<endl;
    cerr<<"RevoluteJoint::propagateX, jointTransform = "<<m_jointTransform<<endl;*/
/*    cerr<<"RevoluteJoint::propagateX, worldTransform = "<<worldTransform<<endl;*/
	// spatial dof
    sp_Si = worldTransform * SpatialVector( Vec(1,0,0), Vec(0,0,0) );
/*    cerr<<"RevoluteJoint::propagateX, sp_Si = "<<sp_Si<<endl;*/
    // spatial velocity
    if( getParent() ){
/*        cerr<<"RevoluteJoint::propagateX, parentVelocity = "<<getParent()->getVelocityInWorld()<<endl;*/
        sp_Vi = getParent()->getVelocityInWorld() + sp_Si * angularVelocity;
    }
    else sp_Vi = sp_Si * angularVelocity;
/*    cerr<<"RevoluteJoint::propagateX, world velocity sp_Vi = "<<sp_Vi<<endl;*/
    sp_Ia = getChild()->getInertia() * worldTransform ;
    //cerr<<"RevoluteJoint::propagateX, sp_Ia = "<<sp_Ia<<endl;
	
    // bias force : force necessary to obtain a null acceleration
    sp_Pi = sp_Vi.cross( sp_Ia * sp_Vi ); // + chargement externe
/*    cerr<<"RevoluteJoint::propagateX, velocity bias force sp_Pi = "<<sp_Pi<<endl;*/
    
    getChild()->setTransform( solidTransform );

/*    cerr<<"RevoluteJoint::propagateX, end---------------------------------- "<<endl;*/
}

void RevoluteJoint::accumulateForce()
{
/*    cerr<<"RevoluteJoint::accumulateForce, start---------------------------------- "<<endl;*/
        
    Real& angularVelocity = getV()[0];
	
    sp_Hi = sp_Ia * sp_Si;
/*    cerr<<"RevoluteJoint::accumulateForce, sp_Hi = "<<sp_Hi<<endl;*/
    sp_Di = sp_Si * sp_Hi;
/*    cerr<<"RevoluteJoint::accumulateForce, sp_Di = "<<sp_Di<<endl;*/
    sp_Ci = sp_Vi.cross( sp_Si*angularVelocity );
/*    cerr<<"RevoluteJoint::accumulateForce, sp_Ci = "<<sp_Ci<<endl;*/
    

    Real& sp_Ui = getF()[0];
    sp_Ui = - this->sp_Hi * this->sp_Ci - this->sp_Si * this->sp_Pi;
/*    cerr<<"RevoluteJoint::accumulateForce, , sp_Ia = "<<sp_Ia<<endl;
    cerr<<"RevoluteJoint::accumulateForce, , sp_Hi = "<<sp_Hi<<endl;
    cerr<<"RevoluteJoint::accumulateForce, , sp_Ci = "<<sp_Ci<<endl;
    cerr<<"RevoluteJoint::accumulateForce, , sp_Si = "<<sp_Si<<endl;
    cerr<<"RevoluteJoint::accumulateForce, , sp_Pi = "<<sp_Pi<<endl;
    cerr<<"RevoluteJoint::accumulateForce, , sp_Ui = "<<sp_Ui<<endl;*/
    
    if( getJointOfParent() ){
        getJointOfParent()->addToIa( getHandleInertia() );
        getJointOfParent()->addToBias( getHandleBias() );
    }
    
/*    cerr<<"RevoluteJoint::accumulateForce, end---------------------------------- "<<endl;*/
}

RevoluteJoint::ArticulatedInertia RevoluteJoint::getHandleInertia()
{
    return sp_Ia - SolidTypes::dyad(sp_Hi,sp_Hi) * (1/sp_Di);
}

RevoluteJoint::SpatialVector RevoluteJoint::getHandleBias()
{
    const Real& sp_Ui = getF()[0];
/*    cerr<<"RevoluteJoint::getHandleBias, sp_Ui = "<<sp_Ui<<endl;
    cerr<<"RevoluteJoint::getHandleBias, sp_Di = "<<sp_Di<<endl;
    cerr<<"RevoluteJoint::getHandleBias, sp_Pi = "<<sp_Pi<<endl;
    cerr<<"RevoluteJoint::getHandleBias, sp_Ia = "<<sp_Ia<<endl;
    cerr<<"RevoluteJoint::getHandleBias, sp_Ci = "<<sp_Ci<<endl;
    cerr<<"RevoluteJoint::getHandleBias, sp_Hi = "<<sp_Hi<<endl;
    cerr<<"RevoluteJoint::getHandleBias, handle bias = "<< this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * (sp_Ui/sp_Di) <<endl;*/
    return this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * (sp_Ui/sp_Di);
}

RevoluteJoint::SpatialVector RevoluteJoint::getSpatialAcceleration()
{
    Real& ui = getF()[0];
    Real& angularAcceleration = getDx()[0];
    SpatialVector parent_sp_Ai;
    if( getParent()!= NULL ){
        parent_sp_Ai = getParent()->get_sp_Ai();
/*        cerr<<"RevoluteJoint::getSpatialAcceleration, parent_sp_Ai = "<<parent_sp_Ai<<endl;*/
    }
    else {
        parent_sp_Ai =  SpatialVector( Vec(0,0,0), -getContext()->getGravityInWorld() );
/*        cerr<<"RevoluteJoint::getSpatialAcceleration, no parent, parent_sp_Ai = "<<parent_sp_Ai<<endl;*/
    }
    
    angularAcceleration = (ui - sp_Hi * parent_sp_Ai)/this->sp_Di ;
/*    cerr<<"RevoluteJoint::getSpatialAcceleration, sp_Di = "<<sp_Di<<endl;
    cerr<<"RevoluteJoint::getSpatialAcceleration, sp_Hi = "<<sp_Hi<<endl;
    cerr<<"RevoluteJoint::getSpatialAcceleration, sp_Hi * parent_sp_Ai = "<<sp_Hi * parent_sp_Ai <<endl;
    cerr<<"RevoluteJoint::getSpatialAcceleration, angularAcceleration = "<<angularAcceleration<<endl;*/
    return this->sp_Ci + sp_Si * angularAcceleration;
}

}//component
}//sofa


