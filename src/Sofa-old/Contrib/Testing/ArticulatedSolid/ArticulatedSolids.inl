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

namespace Sofa {

namespace Components {

  template<class T>
ArticulatedSolid<T>::ArticulatedSolid()
  : Sofa::Abstract::BehaviorModel()
{
}

template<class T>
    ArticulatedSolid<T>::~ArticulatedSolid()
{
}

template<class T>
    void ArticulatedSolid<T>::updatePosition(double dt)
    
{
  cout<<"ArticulatedSolid<T>::updatePosition "<<dt << endl;
}

}// Components
}// Sofa


