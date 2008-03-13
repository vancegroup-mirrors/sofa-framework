//
// C++ Implementation: ArticulatedSolid
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ArticulatedSolid.h"
#include <Sofa-old/Components/Scene.h>
#include <Sofa-old/Components/GL/Repere.h>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

namespace Sofa
{

namespace Components
{

template<class T>
ArticulatedSolid<T>::ArticulatedSolid(std::string /*s*/)
    //: Sofa::Core::DynamicModel()
    : Sofa::Components::RigidObject()
	, _parent(NULL)
                /*    , _parentJoint(NULL)*/
{
        //cout<<"ArticulatedSolid<T>::ArticulatedSolid with argument "<< s << endl;
        this->_intraLinkTransform.clear();
        this->_jointTransform.clear();
        this->_worldTransform.clear();
	this->sp_I.m = 1;
	this->sp_I.h = Vec(0,0,0);
	for( int i=0; i<3; i++ )
		for( int j=0; j<3; j++ )
			sp_I.I[i][j] = i==j ? 1 : 0;
}

template<class T>
ArticulatedSolid<T>::~ArticulatedSolid()
{}

template<class T>
		void ArticulatedSolid<T>::init()
{
//	cerr<<"ArticulatedSolid<T>::init()"<<endl;
	if( this->_parent != NULL ){
		this->worldTransform() = this->_parent->worldTransform() * this->_intraLinkTransform * this->_jointTransform;
		//cerr<<"ArticulatedSolid<T>::init() with parent"<<endl;
	}
	else {
		this->worldTransform() = this->_intraLinkTransform * this->_jointTransform;
		//cerr<<"ArticulatedSolid<T>::init() without parent"<<endl;
	}
	
	for( unsigned i=0; i<_children.size(); ++i )
		_children[i]->init();
		
}

template<class T>
	void ArticulatedSolid<T>::setIntraLinkFrame( const Vec& center, const Vec& axis, Real angle )
{
    Real norm = sqrt(axis*axis);
    assert( norm > 1.0e-5 );
    _intraLinkTransform.setTranslationRotation( center, Rot::createFromRotationVector( axis * (angle / norm) ) );
}

template<class T>
	void ArticulatedSolid<T>::setInertia( Real m, const Vec& c, Real xx, Real yy, Real zz, Real xy, Real yz, Real zx )
{
    sp_I.m = m;
    sp_I.h = c;
    sp_I.I[0][0] = xx; sp_I.I[0][1] = xy; sp_I.I[0][2] = zx;
    sp_I.I[1][0] = xy; sp_I.I[1][1] = yy; sp_I.I[1][2] = yz;
    sp_I.I[2][0] = zx; sp_I.I[2][1] = yz; sp_I.I[2][2] = zz;
	
    // set Inertia at origin rather than mass center
    sp_I.I += T::crossM(sp_I.h)*sp_I.m*T::crossM(sp_I.h).transposed();
	// set mass center
    sp_I.h *= sp_I.m;
}


template<class T>
void ArticulatedSolid<T>::addChild( ArticulatedSolid* s)
{
        _children.push_back(s);
        s->_parent = this;
}



template<class T>
void ArticulatedSolid<T>::execute(Opcode operation, VecId res, VecId a, VecId b, double f)
{
        bool err=false;
        //std::cout << "ArticulatedSolid<T>:: Executing operation("<<getOpcodeName(operation)<<','<< res<<','<< a<<','<< b<<','<< f<<")"<< std::endl;
        switch(operation) {
        case OP_NOP:
                break;
        case OP_BEGIN_ITERATION:
                beginIteration(f);
                break;
        case OP_END_ITERATION:
                endIteration(f);
                break;
        case OP_ALLOC:                 ///< res = new Vector
                break;
        case OP_FREE:                  ///< delete res
                break;
        case OP_CLEAR:                 ///< res = 0
                if (res.type == V_COORD)
                        clearVecCoord(res.index);
                else if (res.type == V_DERIV)
                        clearVecDeriv(res.index);
                else
                        err = true;
                break;
        case OP_EQ:                    ///< res = a
                if (res.type == V_COORD && a.type == V_COORD)
                  vcoord_eq_vcoord(res.index,a.index);
                else if (res.type == V_DERIV && a.type == V_DERIV)
                  vderiv_eq_vderiv(res.index,a.index);
                else
                        err = true;
                break;
        case OP_PEQ:                   ///< res += a*f
                if (res.type == V_COORD && a.type == V_COORD) {
                        cerr<<"ArticulatedSolid<T>::execute, can not perform the product of Transform * scalar"<<endl;
                        err = true;
                } else if (res.type == V_DERIV && a.type == V_DERIV)
                  vderiv_peq_vderiv_times_scalar(res.index,a.index,f);
                else if (res.type == V_COORD && a.type == V_DERIV) {
                  vcoord_peq_vderiv_times_scalar(res.index,a.index,f);
 /*                       cerr<<"ArticulatedSolid<T>::execute, can not perform the product of Transform += SpatialVelocity * scalar"<<endl;
                        err = true;*/
                } else
                        err = true;
                break;
        case OP_TEQ:                   ///< res *= f
                if (res.type == V_COORD) {
                        cerr<<"ArticulatedSolid<T>::execute, can not perform the product of Transform * scalar"<<endl;
                        err = true;
                } else if (res.type == V_DERIV)
                  vderiv_teq_scalar(res.index,f);
                else
                        err = true;
                break;
        case OP_DOT:                   ///< return a dot b
                if (a.type == V_COORD && b.type == V_COORD) {
                        //dest->v_dot(dest->getVecCoord(a.index), dest->getVecCoord(b.index));
                        cerr<<"ArticulatedSolid<T>::execute, can not perform the dot product of Transforms"<<endl;
                        err = true;
                } else if (a.type == V_DERIV && b.type == V_DERIV)
                        compute_vderiv_dot_vderiv(a.index, b.index);
                else
                        err = true;
                break;
        case OP_RESET_FORCE:        ///< force = res = 0
                if (res.type == V_DERIV)
/*                        getVecDeriv(res.index)->clear();*/
                  clearVecDeriv(res.index);
                else
                        err = true;
                break;
        case OP_COMPUTE_FORCE:        ///< res = force
                if (res.type == V_DERIV)
                        computeForce(res.index);
                else
                        err = true;
                break;
        case OP_PROPAGATE_DX:          ///< dx = res
                if (a.type == V_DERIV)
                        propagateDx(a.index);
                else
                        err = true;
                break;
        case OP_COMPUTE_DF:            ///< res = df/dx (dx)
                if (res.type == V_DERIV)
                        computeDf(res.index);
                else
                        err = true;
                break;
        case OP_APPLY_CONSTRAINTS:     ///< res = project(res)
                if (res.type == V_DERIV)
                        applyConstraints(res.index);
                else
                        err = true;
                break;
        case OP_ADD_M_DX:              ///< res = M a
                if (res.type == V_DERIV && a.type == V_DERIV)
                        addMDx(res.index, a.index);
                else
                        err = true;
                break;
        case OP_INTEGRATE_VELOCITY:    ///< res = x + v*dt (where x=a, v=b and dt=f)
                if (res.type == V_COORD && a.type == V_COORD && b.type == V_DERIV)
                        integrateVelocity(res.index, a.index, b.index, f);
                else
                        err = true;
                break;
        case OP_ACC_FROM_F:            ///< x" = M^-1 f (where x"=res and f=a)
                if (res.type == V_DERIV && a.type == V_DERIV)
                        accFromF(res.index,a.index);
                else
                        err = true;
                break;
        case OP_PROPAGATE_POSITION_AND_VELOCITY: ///< update x and v (where x=a and v=b)
                if (a.type == V_COORD && b.type == V_DERIV)
                        propagatePositionAndVelocity(a.index,b.index);
                else
                        err = true;
                break;
        default:
	    Sofa::Components::RigidObject::execute( operation, res, a, b, f);
        }
        if( err )
          cerr<<"ArticulatedSolid<T>::execute, can not perform the operation("<< getOpcodeName(operation)<<','<< res<<','<< a<<','<< b<<','<< f<<")"<< std::endl;
}

template<class T>
    void ArticulatedSolid<T>::computeForce( unsigned result )
{
  for( unsigned i=0; i<_children.size(); ++i )
  {
    _children[i]->computeForce(result);
  }
  /// @todo implement me
}

template<class T>
    void ArticulatedSolid<T>::propagateDx( unsigned dx )
{
  for( unsigned i=0; i<_children.size(); ++i )
  {
    _children[i]->propagateDx(dx);
  }
  /// @todo implement me
}

template<class T>
    void ArticulatedSolid<T>::computeDf( unsigned df )
{
  for( unsigned i=0; i<_children.size(); ++i )
  {
    _children[i]->computeDf(df);
  }
  /// @todo implement me
}

template<class T>
    void ArticulatedSolid<T>::applyConstraints( unsigned dx )
{
  for( unsigned i=0; i<_children.size(); ++i )
  {
    _children[i]->applyConstraints(dx);
  }
  /// @todo implement me
}

template<class T>
    void ArticulatedSolid<T>::addMDx( unsigned df, unsigned dx )
{
  for( unsigned i=0; i<_children.size(); ++i )
  {
    _children[i]->addMDx(df,dx);
  }
  /// @todo implement me
}




template<class T>
double ArticulatedSolid<T>::finish()
{
        double r = lastResult;
        lastResult = 0.0;
        return r;
}

template<class T>
void ArticulatedSolid<T>::setObject(BehaviorModel* /*obj*/)
{
        //cout<<"ArticulatedSolid<T>::setObject"<<endl;
}

template<class T>
void ArticulatedSolid<T>::draw()
{
        if (!Scene::getInstance()->getShowBehaviorModels())
                return;
	double m[16];
	worldTransform().writeOpenGlMatrix(m);
	static GL::Axis *axis = new GL::Axis;
	axis->update(m);
        axis->draw(); 
}



}// Components
}// Sofa


