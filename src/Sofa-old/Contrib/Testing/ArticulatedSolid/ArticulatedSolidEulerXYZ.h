//
// C++ Interface: ArticulatedSolidEulerXYZ
//
// Description: 
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef Sofa_ComponentsArticulatedSolidEulerXYZ_h
#define Sofa_ComponentsArticulatedSolidEulerXYZ_h

#include <ArticulatedSolid.h>
#include <Sofa-old/Components/Common/fixed_array.h>
#include <Sofa-old/Components/Common/vector.h>

namespace Sofa {

namespace Components {

/**
ArticulatedSolid with rotations defined using Euler angles. Translation wrt parent is defined using a standard Vec3. Then, from parent to child, a rotation along X then a rotation along Y then a rotation along Z are performed. It is not advised to use the rotation along Z, since rotations defined using three Euler angles have well-known numerical problems when large rotations occur. For joints with three rotations it is more secure to use a quaternion. 

@author The SOFA team
*/
template<class T>
  class ArticulatedSolidEulerXYZ : public Sofa::Components::ArticulatedSolid<T>
{
public:
    typedef Sofa::Components::ArticulatedSolid<T> Inherited;
    typedef typename T::Real Real;
  typedef typename T::DOF Coord;
  typedef typename T::DOF Deriv;
  typedef typename T::Vec Vec;
  typedef typename T::Rot Rot;
  typedef typename T::Transform Transform;
  
  ArticulatedSolidEulerXYZ();

    ~ArticulatedSolidEulerXYZ();
    virtual void init();
    
    void setCoordinates( const Coord& c );
    void setVelocities( const Deriv& v );

    void compute_vderiv_dot_vderiv(unsigned a, unsigned b) ///< lastResult = a dot b
    {
      this->lastResult = getDeriv(a) * getDeriv(b);
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->compute_vderiv_dot_vderiv(a,b);
      }
    }
    void clearVecCoord(unsigned a) ///< a=0
    {
      getCoord(a).clear();
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->clearVecCoord(a);
      }
    }
    void clearVecDeriv(unsigned a) ///< a=0
    {
      getDeriv(a).clear();
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->clearVecDeriv(a);
      }
    }
    void vcoord_eq_vcoord(unsigned a, unsigned b) ///< a=b
    {
      getCoord(a) = getCoord(b);
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->vcoord_eq_vcoord(a,b);
      }
    }
    void vderiv_eq_vderiv(unsigned a, unsigned b) ///< a=b
    {
      getDeriv(a) = getDeriv(b);
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->vderiv_eq_vderiv(a,b);
      }
    }
    void vderiv_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f) ///< a=b
    {
      getDeriv(a) += getDeriv(b) * f;
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->vderiv_peq_vderiv_times_scalar(a,b,f);
      }
    }
    void vcoord_peq_vderiv_times_scalar(unsigned a, unsigned b, Real f) ///< a=b
    {
      getCoord(a) += getDeriv(b) * f;
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->vcoord_peq_vderiv_times_scalar(a,b,f);
      }
    }
    void vderiv_teq_scalar(unsigned a, Real f) ///< a=b
    {
      getDeriv(a) *= f;
      for( unsigned i=0; i<this->_children.size(); ++i )
      {
        this->_children[i]->vderiv_teq_scalar(a,f);
      }
    }
    
  protected:
    Common::vector<Coord> coords;
    Common::vector<Deriv> derivs;
    
    Coord& getCoord( unsigned index );
    const Coord& getCoord( unsigned index ) const;
    Deriv& getDeriv( unsigned index );
    const Deriv& getDeriv( unsigned index ) const;
};

}

}

#endif
