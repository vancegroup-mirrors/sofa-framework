//
// C++ Implementation: ArticulatedSolidEulerXYZ
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ArticulatedSolidEulerXYZ.h"
#include <iostream>
using std::cout;
using std::endl;

namespace Sofa
{

namespace Components
{

template<class T>
ArticulatedSolidEulerXYZ<T>::ArticulatedSolidEulerXYZ()
                : Sofa::Components::ArticulatedSolid<T>()
{
  getCoord(0).clear();
  getDeriv(0).clear();
}


template<class T>
	ArticulatedSolidEulerXYZ<T>::~ArticulatedSolidEulerXYZ()
{}

template<class T>
	void ArticulatedSolidEulerXYZ<T>::init()
{
    Coord& c= this->getCoord(0);
    this->_jointTransform = Transform( Rot::createFromRotationVector(Vec(c[0],c[1],c[2]) ), Vec(c[3],c[4],c[5]) );
    Inherited::init();

}

template<class T>
	void ArticulatedSolidEulerXYZ<T>::setCoordinates( const Coord& c )
{
    getCoord(0)=c;
    //cout<<"ArticulatedSolidEulerXYZ<T>::setCoordinates "<< c <<endl;
}

template<class T>
	void ArticulatedSolidEulerXYZ<T>::setVelocities( const Deriv& v )
{
    getDeriv(0)=v;
//    cout<<"ArticulatedSolidEulerXYZ<T>::setVelocities "<< v <<endl;
}

template<class T>
	typename ArticulatedSolidEulerXYZ<T>::Coord& ArticulatedSolidEulerXYZ<T>::getCoord( unsigned index )
{
    if (index>=coords.size())
	coords.resize(index+1);
    return coords[index];
}
template<class T>
	const typename ArticulatedSolidEulerXYZ<T>::Coord& ArticulatedSolidEulerXYZ<T>::getCoord( unsigned index ) const
{
    assert(index<coords.size());
    return coords[index];
}

template<class T>
	typename ArticulatedSolidEulerXYZ<T>::Deriv& ArticulatedSolidEulerXYZ<T>::getDeriv( unsigned index )
{
    if (index>=derivs.size())
	derivs.resize(index+1);
    return derivs[index];
}
template<class T>
	const typename ArticulatedSolidEulerXYZ<T>::Deriv& ArticulatedSolidEulerXYZ<T>::getDeriv( unsigned index ) const
{
    assert(index<derivs.size());
    return derivs[index];
}



}//Components

}//Sofa

