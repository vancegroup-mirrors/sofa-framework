//
// C++ Implementation: ArticulatedSolidRevolute
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ArticulatedSolidRevolute.h"
using std::cout;
using std::endl;
//using Sofa::Components::operator <<;

namespace Sofa
{

namespace Components
{

template<class T>
ArticulatedSolidRevolute<T>::ArticulatedSolidRevolute()
                : Sofa::Components::ArticulatedSolidEulerXYZ<T>()
{
}


template<class T>
ArticulatedSolidRevolute<T>::~ArticulatedSolidRevolute()
{}


template<class T>
void ArticulatedSolidRevolute<T>::propagatePositionAndVelocity( unsigned x, unsigned v )
{
    typedef typename T::Transform Transform;
    typedef typename T::RigidInertia RigidInertia;
    typedef typename T::Rot Rot;

        Real& angle = this->getCoord(x)[0];
        Real& angularVelocity = this->getDeriv(v)[0];

        // world transform
	this->_jointTransform = Transform( Rot::createFromRotationVector(Vec(angle,0,0) ), Vec(0,0,0) );
        if( this->_parent != NULL )
                this->worldTransform() = this->_parent->worldTransform() * this->_intraLinkTransform * this->_jointTransform;
        else
                this->worldTransform() = this->_intraLinkTransform * this->_jointTransform;
/*	cout<<"ArticulatedSolidRevolute<T>::propagatePositionAndVelocity, worldTransform = ";
	this->worldTransform().print(cout);
	cout<<endl;*/
        
	
	// spatial dof
        sp_Si = this->worldTransform() * SpatialVector( Vec(1,0,0), Vec(0,0,0) );
//         cout<<"ArticulatedSolidRevolute<T>::propagatePositionAndVelocity, sp_Si = ";
//         sp_Si.print(cout);
//         cout<< endl;

        // spatial velocity
        if( this->_parent != NULL )
                this->sp_Vi = this->_parent->worldVelocity() + sp_Si * angularVelocity;
        else
                this->sp_Vi = sp_Si * angularVelocity;
//         cout<<"ArticulatedSolidRevolute<T>::propagatePositionAndVelocity, world velocity = ";
//         this->sp_Vi.print(cout);
//         cout<< endl;

        // articulated body inertia
/*        cout<<"ArticulatedSolidRevolute<T>::, sp_I = ";
	this->sp_I.print(cout);
	cout<<endl;
	;
	RigidInertia Ri = this->sp_I * this->worldTransform() ;
	cout<<"ArticulatedSolidRevolute<T>::, sp_I projected = ";
	Ri.print(cout);
	cout<<endl;*/
	this->sp_Ia = this->sp_I * this->worldTransform() ;
	
	// bias force
	this->sp_Pi = this->sp_Vi.cross( this->sp_Ia * this->sp_Vi );
//         cout<<"ArticulatedSolidRevolute<T>::propagatePositionAndVelocity, velocity bias force = ";
//         this->sp_Pi.print(cout);
//         cout<<endl;


        // propagate to the children
        for( unsigned i=0; i<this->_children.size(); ++i ) {
                this->_children[i]->propagatePositionAndVelocity(x,v);
        }



	
	for( unsigned i=0; i<this->_children.size(); ++i ) {
	    //this->_children[i]->addHandleInertiaAndBias( this->sp_Ia, this->sp_Pi );
	    this->sp_Ia += this->_children[i]->getHandleInertia();
	}
/*        cout<<"ArticulatedSolidRevolute<T>::, sp_Ia = ";
        this->sp_Ia.print(cout);
        cout<<endl;
        ;*/
        
	
	this->sp_Hi = this->sp_Ia * this->sp_Si;
/*        cout<<"ArticulatedSolidRevolute<T>::, sp_Hi = ";
        this->sp_Hi.print(cout);
        cout<<endl;*/
        ;
        this->sp_Di = this->sp_Si * this->sp_Hi;
/*        cout<<"ArticulatedSolidRevolute<T>::, sp_Di = "<<this->sp_Di<<endl;*/
        ;
        this->sp_Ci = this->sp_Vi.cross( sp_Si*angularVelocity );
/*        cout<<"ArticulatedSolidRevolute<T>::, sp_Ci = ";
        this->sp_Ci.print(cout);
        cout<<endl;*/
        ;
}

// template<class T>
// 	void ArticulatedSolidRevolute<T>::addHandleInertiaAndBias( ArticulatedInertia& inertia, SpatialVector& bias)
// {
//     inertia += this->sp_Ia - ArticulatedSolidTypes<Real>::dyad(sp_Hi,sp_Hi) * (1/sp_Di);
//     bias += this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * (sp_Ui/sp_Di);
// }

template<class T>
	typename  T::ArticulatedInertia ArticulatedSolidRevolute<T>::getHandleInertia() const
{
    return this->sp_Ia - ArticulatedSolidTypes<Real>::dyad(sp_Hi,sp_Hi) * (1/sp_Di);
}

template<class T>
	typename  T::SpatialVector ArticulatedSolidRevolute<T>::getHandleBias(unsigned f) const
{
    const Real& sp_Ui = this->getDeriv(f)[0];
    return this->sp_Pi + this->sp_Ia * this->sp_Ci + sp_Hi * (sp_Ui/sp_Di);
}


template<class T>
	void ArticulatedSolidRevolute<T>::computeForce( unsigned f )
{
    for( unsigned i=0; i<this->_children.size(); ++i )
    {
	this->_children[i]->computeForce(f);
	this->sp_Pi += this->_children[i]->getHandleBias(f);
    }
    Real& sp_Ui = this->getDeriv(f)[0];
    sp_Ui = - this->sp_Hi * this->sp_Ci - this->sp_Si * this->sp_Pi;
    /*        cout<<"ArticulatedSolidRevolute<T>::, sp_Ui = "<<this->sp_Ui<<endl;*/
    ;
}


template<class T>
void ArticulatedSolidRevolute<T>::accFromF( unsigned a, unsigned f )
{
    Real& angularAcceleration = this->getDeriv(a)[0];
    Real& ui = this->getDeriv(f)[0];
    SpatialVector parent_sp_Ai = this->_parent != NULL ? this->_parent->get_sp_Ai() : SpatialVector( Vec(0,0,0), Vec(0,0,1));
	//SpatialVector parent_sp_Ai = this->_parent != NULL ? this->_parent->get_sp_Ai() : SpatialVector( Vec(0,0,0), Vec(0,0,0));
	angularAcceleration = (ui - sp_Hi * parent_sp_Ai)/this->sp_Di ;
        this->sp_Ai = parent_sp_Ai + this->sp_Ci + sp_Si * angularAcceleration;
/*        cout<<"ArticulatedSolidRevolute<T>::accFromF, parent_sp_Ai = ";
        parent_sp_Ai.print(cout);
        cout<<endl;*/
        ;
/*        cout<<"ArticulatedSolidRevolute<T>::accFromF, sp_Hi = ";
        this->sp_Hi.print(cout);
       cout<<endl;*/
        ;
/*	cout<<"ArticulatedSolidRevolute<T>::accFromF, sp_Hi * parent_sp_Ai = "<< sp_Hi * parent_sp_Ai<<endl;
        cout<<"ArticulatedSolidRevolute<T>::accFromF, angularAcceleration = "<<angularAcceleration<<endl;*/
        for( unsigned i=0; i<this->_children.size(); ++i ) {
                this->_children[i]->accFromF(a,f);
        }
}

template<class T>
void ArticulatedSolidRevolute<T>::integrateVelocity( unsigned res, unsigned a, unsigned b, Real dt )
{
        for( unsigned i=0; i<this->_children.size(); ++i ) {
                this->_children[i]->integrateVelocity(res,a,b,dt);
        }
        /// @todo implement me
        this->getCoord(res)[0] = this->getCoord(a)[0] + this->getDeriv(b)[0] * dt;
}


}//Components

}//Sofa

