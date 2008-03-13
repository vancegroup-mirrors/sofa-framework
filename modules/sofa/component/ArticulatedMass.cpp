// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution

#include <sofa/component/ArticulatedMass.h>
#include <sofa/component/ArticulatedBody.h>
#include <sofa/core/componentmodel/behavior/Mass.inl>

namespace sofa
{

namespace component
{

ArticulatedMass::ArticulatedMass(ArticulatedBody* s)
                : body_(s)
{
    body_->setMassNode(this);    
    this->sp_I.m = 1;
        this->sp_I.h = Vec(0,0,0);
        for( int i=0; i<3; i++ )
                for( int j=0; j<3; j++ )
                        sp_I.I[i][j] = i==j ? 1.0f : 0.0f;
}

void ArticulatedMass::setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx )
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

void ArticulatedMass::addMDx(VecDeriv& /*f*/, const VecDeriv& /*dx*/)
{}

void ArticulatedMass::accFromF(VecDeriv& /*a*/, const VecDeriv& /*f*/)
{}

void ArticulatedMass::computeForce(VecDeriv /*f*/, const VecCoord& /*x*/, const VecDeriv& /*v*/)
{
	SolidTypes::Transform localToWorld;
	localToWorld = body_->getContext()->getLocalFrame();
    sp_Ia = sp_I * localToWorld; //body_->getContext()->getLocalToWorld() ;
    
        // bias force
    *body_->getF() += body_->getV()->cross( sp_Ia * (*body_->getV()) );
        //         cout<<"ArticulatedBodyRevolute::propagatePositionAndVelocity, velocity bias force = ";
        //         this->sp_Pi.print(cout);
        //         cout<<endl;
    // 


}

void ArticulatedMass::computeDf(VecDeriv /*df*/, const VecCoord& /*x*/, const VecDeriv& /*v*/, const VecDeriv& /*dx*/)
{}



}//component
}//sofa

