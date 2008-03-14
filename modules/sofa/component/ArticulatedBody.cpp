//
// C++ Implementation: ArticulatedBody
//
// Description:
//
//
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <sofa/component/ArticulatedBody.h>
#include <sofa/core/componentmodel/behavior/ForceField.inl>
#include <sofa/core/componentmodel/behavior/Mass.inl>
#include <sofa/component/Joint.h>
#include "ArticulatedBody.h"  

#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

ArticulatedBody::ArticulatedBody()
        : joint_(NULL)
        , parentBody_(NULL)
{
    // Mass
    this->sp_I.m = 1;
    this->sp_I.h = Vec(0,0,0);
    for( int i=0; i<3; i++ )
        for( int j=0; j<3; j++ )
            sp_I.I[i][j] = i==j ? 1.0f : 0.0f;
}


ArticulatedBody::~ArticulatedBody()
{}

void ArticulatedBody::init()
{
    core::componentmodel::behavior::Mass<core::objectmodel::BaseContext::SolidTypes>::init();
    CoordinateSystem::init();
    
    // MechanicalState
    setX(VecId::position());
    setV(VecId::velocity());
    setF(VecId::force());
    setDx(VecId::dx());

    getX()->clear();
    *getV() = SpatialVector( Vec(0,0,0), Vec(0,0,0) );
    setVelocity( *getV() );

    x0_ = *getX();
    v0_ = *getV();

}

void ArticulatedBody::reset()
{
    getJoint()->resetValues();
    setTransform( getJoint()->getPosition() );
    setVelocity( getJoint()->getVelocity() );
}

void ArticulatedBody::setJoint( Joint* j )
{
    joint_ = j;
    parentBody_ = j->getParent();
    
}

ArticulatedBody* ArticulatedBody::getParent()
{
    return parentBody_;
}

Joint* ArticulatedBody::getJoint()
{
    return joint_;
}

const ArticulatedBody::Frame& ArticulatedBody::getPositionInWorld() const
{
    return BaseObject::getContext()->getPositionInWorld();
}

const ArticulatedBody::SpatialVector& ArticulatedBody::getVelocityInWorld() const
{
    return BaseObject::getContext()->getVelocityInWorld();
}

void ArticulatedBody::setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx )
{
    sp_I.m = m;
    sp_I.h = c;
    sp_I.I[0][0] = xx;
    sp_I.I[0][1] = xy;
    sp_I.I[0][2] = zx;
    sp_I.I[1][0] = xy;
    sp_I.I[1][1] = yy;
    sp_I.I[1][2] = yz;
    sp_I.I[2][0] = zx;
    sp_I.I[2][1] = yz;
    sp_I.I[2][2] = zz;

    // set Inertia at origin rather than mass center
    sp_I.I += SolidTypes::crossM(sp_I.h)*sp_I.m * SolidTypes::crossM(sp_I.h).transposed();
    // set mass center
    sp_I.h *= sp_I.m;
}

const ArticulatedBody::RigidInertia& ArticulatedBody::getInertia() const
{
    return sp_I;
}

double ArticulatedBody::getMass() const
{
    return sp_I.m;
}

void ArticulatedBody::setTransform( const Frame& f )
{
    *getX() = f;
    CoordinateSystem::setTransform(f);
}

void ArticulatedBody::setVelocity( const SpatialVector& v )
{
    *getV()= v ;
}

void ArticulatedBody::setAngularVelocity( const Vec& v )
{
    getV()->setAngularVelocity( v );
}

void ArticulatedBody::setLinearVelocity( const Vec& v )
{
    getV()->setLinearVelocity( v );
}

const ArticulatedBody::SpatialVector& ArticulatedBody::getVelocity() const
{
    return *getV();
}

void ArticulatedBody::apply()
{

    core::objectmodel::BaseContext* context = getContext();
    //     cerr<<"ArticulatedBody::apply(), frame = "<<   getName() <<", t="<<context->getTime() << endl;
    //     cerr<<"ArticulatedBody::apply, inherited position in world = "<<context->getPositionInWorld()<<endl;
    //     cerr<<"ArticulatedBody::apply, transform = "<<this->getTransform()<<endl;

    // store parent position and velocity
    Frame parentToWorld = context->getPositionInWorld();
    SpatialVector parentSpatialVelocity = context->getVelocityInWorld();
    Vec parentLinearVelocity = parentSpatialVelocity.getLinearVelocity() ;
    Vec parentAngularVelocity = parentSpatialVelocity.getAngularVelocity() ;
    Vec parentLinearAcceleration = context->getVelocityBasedLinearAccelerationInWorld() ;


    // Velocity induced by the rotation of the parent frame. Local origin is defined in parent frame.
    Vec originInParentProjected = parentToWorld.projectVector(getOrigin());
    Vec vinduced = parentAngularVelocity.cross( originInParentProjected );
    // Acceleration induced by the rotation of the parent frame. Local origin is defined in parent frame.
    Vec ainduced = parentAngularVelocity.cross( vinduced );




    // update context
    defaulttype::Vec3d newLinearAcceleration = parentLinearAcceleration + ainduced;
    SpatialVector newSpatialVelocity ( parentSpatialVelocity + getTransform() * getVelocity() );
    Frame newLocalToWorld = parentToWorld * getTransform();

    context->setVelocityBasedLinearAccelerationInWorld( newLinearAcceleration );
    context->setPositionInWorld( newLocalToWorld );
    context->setVelocityInWorld( newSpatialVelocity );
    //     cerr<<"CoordinateSystem::apply, position in world= "<<context->getPositionInWorld()<<endl;
    //     cerr<<"CoordinateSystem::apply, velocity in world= "<<context->getVelocityInWorld()<<endl;

}

void ArticulatedBody::addMDx(VecDeriv& f, const VecDeriv& dx)
{
    f = sp_I * dx;
    /*    cerr<<"ArticulatedBody::addMDx, dx = "<< dx <<endl;
        cerr<<"ArticulatedBody::addMDx, f = "<< f <<endl;*/
}

void ArticulatedBody::accFromF(VecDeriv& /*a*/, const VecDeriv& /*f*/)
{
    SpatialVector parent_sp_Ai = getParent() != NULL ? getParent()->get_sp_Ai() : SpatialVector( Vec(0,0,0), -getContext()->getGravityInWorld() );
    /*    cerr<<"ArticulatedBody::accFromF, parent_sp_Ai = "<<parent_sp_Ai <<endl;
        cerr<<"ArticulatedBody::accFromF, getJoint()->getSpatialAcceleration() = "<< getJoint()->getSpatialAcceleration()<<endl;*/
    get_sp_Ai() = parent_sp_Ai + getJoint()->getSpatialAcceleration();
}

void ArticulatedBody::addForce(VecDeriv& f, const VecCoord& /*x*/, const VecDeriv& v)
{
    /*    cerr<<"ArticulatedBody::addForce, x = "<<x<<endl;
        cerr<<"ArticulatedBody::addForce, v = "<< v <<endl;*/

    // gravity
    f.getForce() += getContext()->getGravityInWorld() * this->getMass();
    // bias force
    f += v.cross( sp_I * v );
    //         cout<<"ArticulatedBodyRevolute::propagatePositionAndVelocity, velocity bias force = ";
    //         this->sp_Pi.print(cout);
    //         cout<<endl;
    //
    //     cerr<<"ArticulatedBody::addForce, new f = "<< f <<endl;


}

void ArticulatedBody::addDForce(VecDeriv& /*df*/, const VecCoord& /*x*/, const VecDeriv& /*v*/, const VecDeriv& /*dx*/)
{
    /*    cerr<<"ArticulatedBody::addDForce, dx = "<< dx <<endl;
        cerr<<"ArticulatedBody::addDForce, new df = "<< df <<endl;*/
}

double ArticulatedBody::getKineticEnergy( const VecDeriv& v ) 
{
    return v * (sp_I * v);
}

double ArticulatedBody::getPotentialEnergy( const VecCoord& /*x*/)
{
    cerr<<"ArticulatedBody::getPotentialEnergy-not-implemented !!!"<<endl;
    return 0;    
}



void ArticulatedBody::draw()
{

    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glBegin( GL_LINES );
    glColor3f( 1.f,0.f,0.f );
    glVertex3f( 0.f,0.f,0.f );
    glVertex3f( 1.f,0.f,0.f );
    glColor3f( 0.f,1.f,0.f );
    glVertex3f( 0.f,0.f,0.f );
    glVertex3f( 0.f,1.f,0.f );
    glColor3f( 0.f,0.f,1.f );
    glVertex3f( 0.f,0.f,0.f );
    glVertex3f( 0.f,0.f,1.f );
    glEnd();
    glPopAttrib();

    /*        float m[16];
            double d[16];
            getContext()->computeLocalToWorldMatrixRowMajor(d);
            static GL::Axis *axis = new GL::Axis;
            axis->update(d);
            axis->draw();*/
}


void ArticulatedBody::doEulerStep(VecId xto,VecId vto,VecId xfrom,VecId vfrom, VecId acc, double dt)
{
    getJoint()->doEulerStep(xto,vto,xfrom,vfrom,acc,dt);

    // set the frames accordingly
    Frame& fx = *getCoord(xto.index);
    fx = getJoint()->getFrame(xto.index);
    SpatialVector& vx = *getDeriv(vto.index);
    vx = getJoint()->getSpatialVector(vto.index);

    // update postion if necessary
    if( getCoord(xto.index) == getX() )
        setTransform( getJoint()->getIntraLinkFrame() * fx);
}


void ArticulatedBody::vOp(VecId v, VecId a, VecId b, double f)
{
    /*    cerr<<"    ArticulatedBody::vOp(VecId v, VecId a, VecId b, double f) "<<endl;*/
    if( getJoint()==NULL )
    {
        cerr<<"ArticulatedBody::vOp on "<<getName()<<", null jointNode_"<<endl;
        return;
    }
    getJoint()->vOp(v,a,b,f);
    if (v.type == VecId::V_COORD)
    {
        Frame& vv = *getCoord(v.index);
        vv = getJoint()->getFrame(v.index);
        if( getCoord(v.index) == getX() )
            setTransform( getJoint()->getIntraLinkFrame() * vv);
    }
    else
    {
        SpatialVector& vv = *getDeriv(v.index);
        vv = getJoint()->getSpatialVector(v.index);
    }
}


double ArticulatedBody::vDot(VecId a, VecId b)
{
    /*    cerr<<"    ArticulatedBody::vDot = "<<getJoint()->vDot(a,b)<<endl;*/
    return getJoint()->vDot(a,b);
}



ArticulatedBody::Frame* ArticulatedBody::getCoord(unsigned int index)
{
    if (index>=coords_.size())
        coords_.resize(index+1);
    if (coords_[index]==NULL)
        coords_[index] = new Frame;
    return coords_[index];
}
ArticulatedBody::SpatialVector* ArticulatedBody::getDeriv(unsigned int index)
{
    if (index>=derivs_.size())
        derivs_.resize(index+1);
    if (derivs_[index]==NULL)
        derivs_[index] = new SpatialVector;
    return derivs_[index];
}


void ArticulatedBody::setX( VecId i )
{
    getJoint()->setX(i);
    x_=getCoord(i.index);
    //    *getX() = jointNode_->getPosition();
}
ArticulatedBody::Frame* ArticulatedBody::getX()
{
    return x_;
}
const ArticulatedBody::Frame* ArticulatedBody::getX() const
{
    return x_;
}
const ArticulatedBody::Frame* ArticulatedBody::getX0() const
{
    return &x0_;
}

void ArticulatedBody::setV( VecId i )
{
    getJoint()->setV(i);
    v_=getDeriv(i.index);
    //    *getV() = jointNode_->getVelocity();
}
ArticulatedBody::SpatialVector* ArticulatedBody::getV()
{
    return v_;
}
const ArticulatedBody::SpatialVector* ArticulatedBody::getV() const
{
    return v_;
}
const ArticulatedBody::SpatialVector* ArticulatedBody::getV0() const
{
    return &v0_;
}

void ArticulatedBody::setF( VecId i )
{
    getJoint()->setF(i);
    f_=getDeriv(i.index);
    /*    *getF() = jointNode_->getForce();*/
}
ArticulatedBody::SpatialVector* ArticulatedBody::getF()
{
    return f_;
}
const ArticulatedBody::SpatialVector* ArticulatedBody::getF() const
{
    return f_;
}

void ArticulatedBody::setDx( VecId i )
{
    getJoint()->setDx(i);
    dx_=getDeriv(i.index);
    /*    *getDx() = jointNode_->getDisplacement();*/
}
ArticulatedBody::SpatialVector* ArticulatedBody::getDx()
{
    return dx_;
}
const ArticulatedBody::SpatialVector* ArticulatedBody::getDx() const
{
    return dx_;
}

void ArticulatedBody::resetForce()
{
    *getF() = SpatialVector( Vec(0,0,0),Vec(0,0,0) );
}


void ArticulatedBody::printDOF( VecId v, std::ostream& out)
{
    if( v.type==VecId::V_COORD )
        out<<*getCoord(v.index);
    else if( v.type==VecId::V_DERIV )
        out<<*getDeriv(v.index);
    else
        out<<"ArticulatedBody::printDOF, unknown v.type = "<<v.type<<endl;
}

    void ArticulatedBody::getIndicesInSpace(sofa::helper::vector<unsigned>& indices, Real xmin, Real xmax,Real ymin, Real ymax, Real zmin, Real zmax) const
{
    Vec x = getPositionInWorld().getOrigin();
    if( x[0]>=xmin && x[0]<=xmax && x[1]>=ymin && x[1]<=ymax && x[2]>=zmin && x[2]<=zmax )
        indices.push_back(0);
}


}// namespace component

namespace core
{
namespace componentmodel
{
namespace behavior
{
template class ForceField<objectmodel::BaseContext::SolidTypes>;
template class Mass<objectmodel::BaseContext::SolidTypes>;
}
}
}

}// namespace sofa


