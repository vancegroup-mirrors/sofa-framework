/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_DEFAULTTYPE_RIGIDTYPES_H
#define SOFA_DEFAULTTYPE_RIGIDTYPES_H

#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/componentmodel/behavior/Mass.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/rmath.h>
#include <iostream>
using std::endl;

namespace sofa
{

namespace defaulttype
{

using sofa::helper::vector;

template<int N, typename real>
class RigidDeriv;

template<int N, typename real>
class RigidCoord;

template<int N, typename real>
class RigidMass;

template<int N, typename real>
class StdRigidTypes;

//=============================================================================
// 3D Rigids
//=============================================================================

/** Degrees of freedom of 3D rigid bodies. Orientations are modeled using quaternions.
*/
template<typename real>
class RigidDeriv<3, real>
{
public:
	typedef real value_type;
    typedef real Real;
    typedef Vec<3,Real> Vec3;
    typedef helper::Quater<Real> Quat;

protected:
    Vec3 vCenter;
    Vec3 vOrientation;
public:
    friend class RigidCoord<3,real>;

    RigidDeriv(const Vec3 &velCenter, const Vec3 &velOrient)
    : vCenter(velCenter), vOrientation(velOrient) {}
    RigidDeriv() { clear(); }

    template<typename real2>
    RigidDeriv(const RigidDeriv<3,real2>& c)
    : vCenter(c.getVCenter()), vOrientation(c.getVOrientation())
    {
    }

    void clear() { vCenter.clear(); vOrientation.clear(); }

    template<typename real2>
    void operator =(const RigidDeriv<3,real2>& c)
    {
        vCenter = c.getVCenter();
        vOrientation = c.getVOrientation();
    }

    void operator +=(const RigidDeriv& a)
    {
        vCenter += a.vCenter;
        vOrientation += a.vOrientation;
    }

	void operator -=(const RigidDeriv& a)
    {
        vCenter -= a.vCenter;
        vOrientation -= a.vOrientation;
    }

    RigidDeriv<3,real> operator + (const RigidDeriv<3,real>& a) const
    {
        RigidDeriv d;
        d.vCenter = vCenter + a.vCenter;
        d.vOrientation = vOrientation + a.vOrientation;
        return d;
    }

    void operator*=(double a)
    {
        vCenter *= a;
        vOrientation *= a;
    }

    RigidDeriv<3,real> operator*(double a) const
    {
        RigidDeriv r = *this;
        r*=a;
        return r;
    }

    RigidDeriv<3,real> operator - () const 
    {
        return RigidDeriv(-vCenter, -vOrientation);
    }
		 
	RigidDeriv<3,real> operator - (const RigidDeriv<3,real>& a) const
	{
		return RigidDeriv<3,real>(this->vCenter - a.vCenter, this->vOrientation-a.vOrientation);
	}


    /// dot product, mostly used to compute residuals as sqrt(x*x)
    double operator*(const RigidDeriv<3,real>& a) const
    {
        return vCenter[0]*a.vCenter[0]+vCenter[1]*a.vCenter[1]+vCenter[2]*a.vCenter[2]
            +vOrientation[0]*a.vOrientation[0]+vOrientation[1]*a.vOrientation[1]
            +vOrientation[2]*a.vOrientation[2];
    }

    Vec3& getVCenter (void) { return vCenter; }
    Vec3& getVOrientation (void) { return vOrientation; }
    const Vec3& getVCenter (void) const { return vCenter; }
    const Vec3& getVOrientation (void) const { return vOrientation; }
    
	 Vec3& getLinear () { return vCenter; }
	 const Vec3& getLinear () const { return vCenter; }
	 Vec3& getAngular () { return vOrientation; }
	 const Vec3& getAngular () const { return vOrientation; }
	

	 
	 /// write to an output stream
    inline friend std::ostream& operator << ( std::ostream& out, const RigidDeriv<3,real>& v ){
        out<<v.vCenter<<" "<<v.vOrientation;
        return out;
    }
    /// read from an input stream
    inline friend std::istream& operator >> ( std::istream& in, RigidDeriv<3,real>& v ){
        in>>v.vCenter>>v.vOrientation;
        return in;
    }

	static unsigned int size(){return 6;};

	/// Access to i-th element.
    real& operator[](int i)
    {
		if (i<3)
			return this->vCenter(i);
		else
			return this->vOrientation(i-3);
    }

    /// Const access to i-th element.
    const real& operator[](int i) const
    {
		if (i<3)
			return this->vCenter(i);
		else
			return this->vOrientation(i-3);
    }
};

template<typename real>
class RigidCoord<3,real>
{
public:
  typedef real value_type;
    typedef real Real;
    typedef Vec<3,Real> Vec3;
    typedef helper::Quater<Real> Quat;

protected:
    Vec3 center;
    Quat orientation;
public:
    RigidCoord (const Vec3 &posCenter, const Quat &orient)
    : center(posCenter), orientation(orient) {}
    RigidCoord () { clear(); }
    
    template<typename real2>
    RigidCoord(const RigidCoord<3,real2>& c)
    : center(c.getCenter()), orientation(c.getOrientation())
    {
    }
    
    
    void clear() { center.clear(); orientation.clear(); }
    
    template<typename real2>
    void operator =(const RigidCoord<3,real2>& c)
    {
        center = c.getCenter();
        orientation = c.getOrientation();
    }
    
    //template<typename real2>
    //void operator =(const RigidCoord<3,real2>& c)
    //{
    //    center = c.getCenter();
    //    orientation = c.getOrientation();
    //}

    void operator +=(const RigidDeriv<3,real>& a)
    {
        center += a.getVCenter();
        orientation.normalize();
        Quat qDot = orientation.vectQuatMult(a.getVOrientation());
        for (int i = 0; i < 4; i++)
            orientation[i] += qDot[i] * 0.5f;
        orientation.normalize();
    }
    
    RigidCoord<3,real> operator + (const RigidDeriv<3,real>& a) const
    {
        RigidCoord c = *this;
        c.center += a.getVCenter();
        c.orientation.normalize();
        Quat qDot = c.orientation.vectQuatMult(a.getVOrientation());
        for (int i = 0; i < 4; i++)
            c.orientation[i] += qDot[i] * 0.5f;
        c.orientation.normalize();
        return c;
    }

	RigidCoord<3,real> operator -(const RigidCoord<3,real>& a) const
    {
        return RigidCoord<3,real>(this->center - a.getCenter(), a.orientation.inverse() * this->orientation);
    }
    
    void operator +=(const RigidCoord<3,real>& a)
    {
        //std::cout << "+="<<std::endl;
        center += a.getCenter();
        //orientation += a.getOrientation();
        //orientation.normalize();
    }
    
    void operator*=(double a)
    {
        //std::cout << "*="<<std::endl;
        center *= a;
        //orientation *= a;
    }
    
    RigidCoord<3,real> operator*(double a) const
    {
        RigidCoord r = *this;
        r*=a;
        return r;
    }
    
    /// dot product, mostly used to compute residuals as sqrt(x*x)
    double operator*(const RigidCoord<3,real>& a) const
    {
        return center[0]*a.center[0]+center[1]*a.center[1]+center[2]*a.center[2]
            +orientation[0]*a.orientation[0]+orientation[1]*a.orientation[1]
            +orientation[2]*a.orientation[2]+orientation[3]*a.orientation[3];
    }
    
    Vec3& getCenter () { return center; }
    Quat& getOrientation () { return orientation; }
    const Vec3& getCenter () const { return center; }
    const Quat& getOrientation () const { return orientation; }
    
    static RigidCoord<3,real> identity() {
        RigidCoord c;
        return c;
    }
    
    /// Apply a transformation with respect to itself
    void multRight( const RigidCoord<3,real>& c )
    {
        center += orientation.rotate(c.getCenter());
        orientation = orientation * c.getOrientation();
    }
    
    /// compute the product with another frame on the right
    RigidCoord<3,real> mult( const RigidCoord<3,real>& c ) const {
        RigidCoord r;
        r.center = center + orientation.rotate( c.center );
        r.orientation = orientation * c.getOrientation();
        return r;
    }
    
    /// Set from the given matrix
    template<class Mat>
    void fromMatrix(const Mat& m) const
    {
        center[0] = m[0][3];
        center[1] = m[1][3];
        center[2] = m[2][3];
        Mat3x3d rot; rot = m;
        orientation.fromMatrix(rot);
    }
    
    /// Write to the given matrix
    template<class Mat>
    void toMatrix( Mat& m) const
    {
        m.identity();
        orientation.toMatrix(m);
        m[0][3] = center[0];
        m[1][3] = center[1];
        m[2][3] = center[2];
    }
    
    template<class Mat>
    void writeRotationMatrix( Mat& m) const
    {
        orientation.toMatrix(m);
    }
    
    /// Write the OpenGL transformation matrix
    void writeOpenGlMatrix( float m[16] ) const {
        orientation.writeOpenGlMatrix(m);
        m[12] = (float)center[0];
        m[13] = (float)center[1];
        m[14] = (float)center[2];
    }
    
    /// compute the projection of a vector from the parent frame to the child
    Vec3 vectorToChild( const Vec3& v ) const {
        return orientation.inverseRotate(v);
    }
    
    /// write to an output stream
    inline friend std::ostream& operator << ( std::ostream& out, const RigidCoord<3,real>& v ){
        out<<v.center<<" "<<v.orientation;
        return out;
    }
    /// read from an input stream
    inline friend std::istream& operator >> ( std::istream& in, RigidCoord<3,real>& v ){
        in>>v.center>>v.orientation;
        return in;
    }
	static int max_size()
	{
		return 3;
	}
	enum { static_size = 3 };

	static unsigned int size(){return 7;};

	/// Access to i-th element.
    real& operator[](int i)
    {
		if (i<3)
			return this->center(i);
		else
			return this->orientation[i-3];
    }

    /// Const access to i-th element.
    const real& operator[](int i) const
    {
		if (i<3)
			return this->center(i);
		else
			return this->orientation[i-3];
    }
};

template<typename real>
class RigidMass<3, real>
{
public:
	typedef real value_type;
    typedef real Real;
    typedef Mat<3,3,Real> Mat3x3;
    Real mass,volume;
    Mat3x3 inertiaMatrix;	      // Inertia matrix of the object
    Mat3x3 inertiaMassMatrix;    // Inertia matrix of the object * mass of the object
    Mat3x3 invInertiaMatrix;	  // inverse of inertiaMatrix
    Mat3x3 invInertiaMassMatrix; // inverse of inertiaMassMatrix
    RigidMass(Real m=1)
    {
        mass = m;
        volume = 1;
        inertiaMatrix.identity();
        recalc();
    }
    void operator=(Real m)
    {
        mass = m;
        recalc();
    }
    void recalc()
    {
        inertiaMassMatrix = inertiaMatrix * mass;
        invInertiaMatrix.invert(inertiaMatrix);
        invInertiaMassMatrix.invert(inertiaMassMatrix);
    }
    
    inline friend std::ostream& operator << (std::ostream& out, const RigidMass<3, real>& m )
    {
        out<<m.mass;
        out<<" "<<m.volume;
        out<<" "<<m.inertiaMatrix;
        return out;
    }
    inline friend std::istream& operator >> (std::istream& in, RigidMass<3, real>& m )
    {
        in>>m.mass;
        in>>m.volume;
        in>>m.inertiaMatrix;
        return in;
    }
    void operator *=(Real fact)
    {
        mass *= fact;
        inertiaMassMatrix *= fact;
        invInertiaMassMatrix /= fact;
    }
};

template<int N, typename real>
inline RigidDeriv<N,real> operator*(const RigidDeriv<N,real>& d, const RigidMass<N,real>& m)
{
    RigidDeriv<N,real> res;
    res.getVCenter() = d.getVCenter() * m.mass;
    res.getVOrientation() = m.inertiaMassMatrix * d.getVOrientation();
    return res;
}

template<int N, typename real>
inline RigidDeriv<N, real> operator/(const RigidDeriv<N, real>& d, const RigidMass<N, real>& m)
{
    RigidDeriv<N, real> res;
    res.getVCenter() = d.getVCenter() / m.mass;
    res.getVOrientation() = m.invInertiaMassMatrix * d.getVOrientation();
    return res;
}


template<typename real>
class StdRigidTypes<3, real>
{
public:
    typedef real Real;
    typedef RigidCoord<3,real> Coord;
    typedef RigidDeriv<3,real> Deriv;
    typedef typename Coord::Vec3 Vec3;
    typedef typename Coord::Quat Quat;

    template <class T>
    class SparseData
    {
    public:
        SparseData(unsigned int _index, const T& _data): index(_index), data(_data){};
        unsigned int index;
        T data;
    };

    typedef SparseData<Coord> SparseCoord;
    typedef SparseData<Deriv> SparseDeriv;

    typedef vector<SparseCoord> SparseVecCoord;
    typedef vector<SparseDeriv> SparseVecDeriv;

    //! All the Constraints applied to a state Vector
    typedef	vector<SparseVecDeriv> VecConst;
    
    typedef vector<Coord> VecCoord;
    typedef vector<Deriv> VecDeriv;
    typedef vector<Real> VecReal;

    static void set(Coord& c, double x, double y, double z)
    {
        c.getCenter()[0] = (Real)x;
        c.getCenter()[1] = (Real)y;
        c.getCenter()[2] = (Real)z;
    }
    
    static void get(double& x, double& y, double& z, const Coord& c)
    {
        x = c.getCenter()[0];
        y = c.getCenter()[1];
        z = c.getCenter()[2];
    }
    
    static void add(Coord& c, double x, double y, double z)
    {
        c.getCenter()[0] += (Real)x;
        c.getCenter()[1] += (Real)y;
        c.getCenter()[2] += (Real)z;
    }
    
    static void set(Deriv& c, double x, double y, double z)
    {
        c.getVCenter()[0] = (Real)x;
        c.getVCenter()[1] = (Real)y;
        c.getVCenter()[2] = (Real)z;
    }
    
    static void get(double& x, double& y, double& z, const Deriv& c)
    {
        x = c.getVCenter()[0];
        y = c.getVCenter()[1];
        z = c.getVCenter()[2];
    }
    
    static void add(Deriv& c, double x, double y, double z)
    {
        c.getVCenter()[0] += (Real)x;
        c.getVCenter()[1] += (Real)y;
        c.getVCenter()[2] += (Real)z;
    }
    
    static const char* Name();
};

typedef StdRigidTypes<3,double> Rigid3dTypes;
typedef StdRigidTypes<3,float> Rigid3fTypes;
typedef Rigid3dTypes Rigid3Types;
typedef Rigid3Types RigidTypes;

typedef RigidMass<3,double> Rigid3dMass;
typedef RigidMass<3,float> Rigid3fMass;
typedef Rigid3dMass Rigid3Mass;
//typedef Rigid3Mass RigidMass;

/// Note: Many scenes use Rigid as template for 3D double-precision rigid type. Changing it to Rigid3d would break backward compatibility.
template<> inline const char* Rigid3dTypes::Name() { return "Rigid"; }
template<> inline const char* Rigid3fTypes::Name() { return "Rigid3f"; }


//=============================================================================
// 2D Rigids
//=============================================================================

template<typename real>
class RigidDeriv<2,real>
{
public:
	typedef real value_type;
    typedef real Real;
    typedef Vec<2,Real> Vec2;
private:
    Vec2 vCenter;
    Real vOrientation;
public:
    friend class RigidCoord<2,real>;

    RigidDeriv (const Vec2 &velCenter, const Real &velOrient)
    : vCenter(velCenter), vOrientation(velOrient) {}
    RigidDeriv () { clear(); }

    void clear() { vCenter.clear(); vOrientation=0; }

    void operator +=(const RigidDeriv<2,real>& a)
    {
        vCenter += a.vCenter;
        vOrientation += a.vOrientation;
    }

    RigidDeriv<2,real> operator + (const RigidDeriv<2,real>& a) const
    {
        RigidDeriv<2,real> d;
        d.vCenter = vCenter + a.vCenter;
        d.vOrientation = vOrientation + a.vOrientation;
        return d;
    }

    void operator*=(double a)
    {
        vCenter *= (Real)a;
        vOrientation *= (Real)a;
    }

    RigidDeriv<2,real> operator*(double a) const
    {
        RigidDeriv<2,real> r = *this;
        r *= (Real)a;
        return r;
    }

    RigidDeriv<2,real> operator - () const
    {
        return RigidDeriv<2,real>(-vCenter, -vOrientation);
    }

    /// dot product, mostly used to compute residuals as sqrt(x*x)
    double operator*(const RigidDeriv<2,real>& a) const
    {
        return vCenter[0]*a.vCenter[0]+vCenter[1]*a.vCenter[1]
            +vOrientation*a.vOrientation;
    }

    Vec2& getVCenter (void) { return vCenter; }
    Real& getVOrientation (void) { return vOrientation; }
    const Vec2& getVCenter (void) const { return vCenter; }
    const Real& getVOrientation (void) const { return vOrientation; }
    /// write to an output stream
    inline friend std::ostream& operator << ( std::ostream& out, const RigidDeriv<2,real>& v )
    {
        out<<v.vCenter<<" "<<v.vOrientation;
        return out;
    }
    /// read from an input stream
    inline friend std::istream& operator >> ( std::istream& in, RigidDeriv<2,real>& v )
    {
        in>>v.vCenter>>v.vOrientation;
        return in;
    }

	static unsigned int size(){return 3;};

	/// Access to i-th element.
    real& operator[](int i)
    {
		if (i<2)
			return this->vCenter(i);
		else
			return this->vOrientation;
    }

    /// Const access to i-th element.
    const real& operator[](int i) const
    {
		if (i<2)
			return this->vCenter(i);
		else
			return this->vOrientation;
    }
};

template<typename real>
class RigidCoord<2,real>
{
public:
	typedef real value_type;
    typedef real Real;
    typedef Vec<2,Real> Vec2;
private:
    Vec2 center;
    Real orientation;
public:
    RigidCoord (const Vec2 &posCenter, const Real &orient)
    : center(posCenter), orientation(orient) {}
    RigidCoord () { clear(); }

    void clear() { center.clear(); orientation = 0; }

    void operator +=(const RigidDeriv<2,real>& a)
    {
        center += a.getVCenter();
        orientation += a.getVOrientation();
    }

    RigidCoord<2,real> operator + (const RigidDeriv<2,real>& a) const
    {
        RigidCoord<2,real> c = *this;
        c.center += a.getVCenter();
        c.orientation += a.getVOrientation();
        return c;
    }

    void operator +=(const RigidCoord<2,real>& a)
    {
        std::cout << "+="<<std::endl;
        center += a.getCenter();
        orientation += a.getOrientation();
    }

    void operator*=(double a)
    {
        std::cout << "*="<<std::endl;
        center *= (Real)a;
        orientation *= (Real)a;
    }

    RigidCoord<2,real> operator*(double a) const
    {
        RigidCoord<2,real> r = *this;
        r *= (Real)a;
        return r;
    }

    /// dot product, mostly used to compute residuals as sqrt(x*x)
    double operator*(const RigidCoord<2,real>& a) const
    {
        return center[0]*a.center[0]+center[1]*a.center[1]
            +orientation*a.orientation;
    }

    Vec2& getCenter () { return center; }
    Real& getOrientation () { return orientation; }
    const Vec2& getCenter () const { return center; }
    const Real& getOrientation () const { return orientation; }

    Vec2 rotate(const Vec2& v) const
    {
        Real s = sin(orientation);
        Real c = cos(orientation);
        return Vec2(c*v[0]-s*v[1],
                    s*v[0]+c*v[1]);
    }
    Vec2 inverseRotate(const Vec2& v) const
    {
        Real s = sin(-orientation);
        Real c = cos(-orientation);
        return Vec2(c*v[0]-s*v[1],
                    s*v[0]+c*v[1]);
    }

    static RigidCoord<2,real> identity()
    {
        RigidCoord<2,real> c;
        return c;
    }

    /// Apply a transformation with respect to itself
    void multRight( const RigidCoord<2,real>& c )
    {
        center += /*orientation.*/rotate(c.getCenter());
        orientation = orientation * c.getOrientation();
    }

    /// compute the product with another frame on the right
    RigidCoord<2,real> mult( const RigidCoord<2,real>& c ) const
    {
        RigidCoord<2,real> r;
        r.center = center + /*orientation.*/rotate( c.center );
        r.orientation = orientation * c.getOrientation();
        return r;
    }

    template<class Mat>
    void writeRotationMatrix( Mat& m) const
    {
        m[0][0] = (typename Mat::Real)cos(orientation); m[0][1] = (typename Mat::Real)-sin(orientation);
        m[1][0] = (typename Mat::Real)sin(orientation); m[1][1] = (typename Mat::Real) cos(orientation);
    }

    /// Set from the given matrix
    template<class Mat>
    void fromMatrix(const Mat& m) const
    {
        center[0] = m[0][2];
        center[1] = m[1][2];
        orientation = atan2(m[1][0],m[0][0]);
    }

    /// Write to the given matrix
    template<class Mat>
    void toMatrix( Mat& m) const
    {
        m.identity();
        writeRotationMatrix( m );
        m[0][2] = center[0];
        m[1][2] = center[1];
    }

    /// Write the OpenGL transformation matrix
    void writeOpenGlMatrix( float m[16] ) const
    {
        //orientation.writeOpenGlMatrix(m);
        m[0] = cos(orientation);
        m[1] = sin(orientation);
        m[2] = 0;
        m[3] = 0;
        m[4] = -sin(orientation);
        m[5] = cos(orientation);
        m[6] = 0;
        m[7] = 0;
        m[8] = 0;
        m[9] = 0;
        m[10] = 1;
        m[11] = 0;
        m[12] = (float)center[0];
        m[13] = (float)center[1];
        m[14] = (float)center[2];
        m[15] = 1;
    }

    /// compute the projection of a vector from the parent frame to the child
    Vec2 vectorToChild( const Vec2& v ) const
    {
        return /*orientation.*/inverseRotate(v);
    }

    /// write to an output stream
    inline friend std::ostream& operator << ( std::ostream& out, const RigidCoord<2,real>& v )
    {
        out<<v.center<<" "<<v.orientation;
        return out;
    }
    /// read from an input stream
    inline friend std::istream& operator >> ( std::istream& in, RigidCoord<2,real>& v )
    {
        in>>v.center>>v.orientation;
        return in;
    }
    static int max_size()
    {
        return 3;
    }
    enum { static_size = 3 };

	static unsigned int size(){return 3;};

	/// Access to i-th element.
    real& operator[](int i)
    {
		if (i<2)
			return this->center(i);
		else
			return this->orientation;
    }

    /// Const access to i-th element.
    const real& operator[](int i) const
    {
		if (i<2)
			return this->center(i);
		else
			return this->orientation;
    }
};

template<class real>
class RigidMass<2, real>
{
public:
	typedef real value_type;
    typedef real Real;
    Real mass,volume;
    Real inertiaMatrix;	      // Inertia matrix of the object
    Real inertiaMassMatrix;    // Inertia matrix of the object * mass of the object
    Real invInertiaMatrix;	  // inverse of inertiaMatrix
    Real invInertiaMassMatrix; // inverse of inertiaMassMatrix
    RigidMass(Real m=1)
    {
        mass = m;
        volume = 1;
        inertiaMatrix = 1;
        recalc();
    }
    void operator=(Real m)
    {
        mass = m;
        recalc();
    }
    /// Mass for a circle
    RigidMass(Real m, Real radius)
    {
        mass = m;
        volume = radius*radius*R_PI;
        inertiaMatrix = (radius*radius)/2;
        recalc();
    }
    /// Mass for a rectangle
    RigidMass(Real m, Real xwidth, Real ywidth)
    {
        mass = m;
        volume = xwidth*xwidth + ywidth*ywidth;
        inertiaMatrix = volume/12;
        recalc();
    }

    void recalc()
    {
        inertiaMassMatrix = inertiaMatrix * mass;
        invInertiaMatrix = 1/(inertiaMatrix);
        invInertiaMassMatrix = 1/(inertiaMassMatrix);
    }
    inline friend std::ostream& operator << (std::ostream& out, const RigidMass<2,Real>& m )
    {
        out<<m.mass;
        out<<" "<<m.volume;
        out<<" "<<m.inertiaMatrix;
        return out;
    }
    inline friend std::istream& operator >> (std::istream& in, RigidMass<2,Real>& m )
    {
        in>>m.mass;
        in>>m.volume;
        in>>m.inertiaMatrix;
        return in;
    }
    void operator *=(Real fact)
    {
        mass *= fact;
        inertiaMassMatrix *= fact;
        invInertiaMassMatrix /= fact;
    }
};

/** Degrees of freedom of 2D rigid bodies.
*/
template<typename real>
class StdRigidTypes<2, real>
{
public:
    typedef real Real;
    typedef Vec<2,real> Vec2;
    
    typedef RigidDeriv<2,Real> Deriv;
    typedef RigidCoord<2,Real> Coord;
    
    static const char* Name();

    typedef vector<Coord> VecCoord;
    typedef vector<Deriv> VecDeriv;
    
    
    template <class T>
    class SparseData
    {
    public:
        SparseData(unsigned int _index, const T& _data): index(_index), data(_data){};
        unsigned int index;
        T data;
    };

    typedef SparseData<Coord> SparseCoord;
    typedef SparseData<Deriv> SparseDeriv;
    typedef vector<Real> VecReal;

    typedef vector<SparseCoord> SparseVecCoord;
    typedef vector<SparseDeriv> SparseVecDeriv;
    
    typedef	vector<SparseVecDeriv> VecConst;
    
    static void set(Coord& c, double x, double y, double)
    {
        c.getCenter()[0] = (Real)x;
        c.getCenter()[1] = (Real)y;
    }

    static void get(double& x, double& y, double& z, const Coord& c)
    {
        x = c.getCenter()[0];
        y = c.getCenter()[1];
        z = 0;
    }

    static void add(Coord& c, double x, double y, double)
    {
        c.getCenter()[0] += (Real)x;
        c.getCenter()[1] += (Real)y;
    }

    static void set(Deriv& c, double x, double y, double)
    {
        c.getVCenter()[0] = (Real)x;
        c.getVCenter()[1] = (Real)y;
    }

    static void get(double& x, double& y, double& z, const Deriv& c)
    {
        x = c.getVCenter()[0];
        y = c.getVCenter()[1];
        z = 0;
    }

    static void add(Deriv& c, double x, double y, double)
    {
        c.getVCenter()[0] += (Real)x;
        c.getVCenter()[1] += (Real)y;
    }

};

typedef StdRigidTypes<2,double> Rigid2dTypes;
typedef StdRigidTypes<2,float> Rigid2fTypes;
typedef Rigid2dTypes Rigid2Types;

typedef RigidMass<2,double> Rigid2dMass;
typedef RigidMass<2,float> Rigid2fMass;
typedef Rigid2dMass Rigid2Mass;

template<> inline const char* Rigid2dTypes::Name() { return "Rigid2d"; }
template<> inline const char* Rigid2fTypes::Name() { return "Rigid2f"; }

} // namespace defaulttype

namespace core
{
namespace componentmodel
{
namespace behavior
{

/** Return the inertia force applied to a body referenced in a moving coordinate system.
\param sv spatial velocity (omega, vorigin) of the coordinate system
\param a acceleration of the origin of the coordinate system
\param m mass of the body
\param x position of the body in the moving coordinate system
\param v velocity of the body in the moving coordinate system
This default implementation returns no inertia.
*/
template<class Coord, class Deriv, class Vec, class M, class SV>
Deriv inertiaForce( const SV& /*sv*/, const Vec& /*a*/, const M& /*m*/, const Coord& /*x*/, const Deriv& /*v*/ );

/// Specialization of the inertia force for defaulttype::Rigid3dTypes
template <>
inline defaulttype::RigidDeriv<3, double> inertiaForce<
defaulttype::RigidCoord<3, double>,
defaulttype::RigidDeriv<3, double>,
objectmodel::BaseContext::Vec3,
defaulttype::RigidMass<3, double>,
objectmodel::BaseContext::SpatialVector
>
(
const objectmodel::BaseContext::SpatialVector& vframe,
const objectmodel::BaseContext::Vec3& aframe,
const defaulttype::RigidMass<3, double>& mass,
const defaulttype::RigidCoord<3, double>& x,
const defaulttype::RigidDeriv<3, double>& v
)
{
    defaulttype::RigidDeriv<3, double>::Vec3 omega( vframe.lineVec[0], vframe.lineVec[1], vframe.lineVec[2] );
    defaulttype::RigidDeriv<3, double>::Vec3 origin = x.getCenter(), finertia, zero(0,0,0);

    finertia = -( aframe + omega.cross( omega.cross(origin) + v.getVCenter()*2 ))*mass.mass;
    return defaulttype::RigidDeriv<3, double>( finertia, zero );
    /// \todo replace zero by Jomega.cross(omega)
}

/// Specialization of the inertia force for defaulttype::Rigid3fTypes
template <>
inline defaulttype::RigidDeriv<3, float> inertiaForce<
defaulttype::RigidCoord<3, float>,
defaulttype::RigidDeriv<3, float>,
objectmodel::BaseContext::Vec3,
defaulttype::RigidMass<3, float>,
objectmodel::BaseContext::SpatialVector
>
(
const objectmodel::BaseContext::SpatialVector& vframe,
const objectmodel::BaseContext::Vec3& aframe,
const defaulttype::RigidMass<3, float>& mass,
const defaulttype::RigidCoord<3, float>& x,
const defaulttype::RigidDeriv<3, float>& v
)
{
    defaulttype::RigidDeriv<3, float>::Vec3 omega( (float)vframe.lineVec[0], (float)vframe.lineVec[1], (float)vframe.lineVec[2] );
    defaulttype::RigidDeriv<3, float>::Vec3 origin = x.getCenter(), finertia, zero(0,0,0);

    finertia = -( aframe + omega.cross( omega.cross(origin) + v.getVCenter()*2 ))*mass.mass;
    return defaulttype::RigidDeriv<3, float>( finertia, zero );
    /// \todo replace zero by Jomega.cross(omega)
}

} // namespace behavoir

} // namespace componentmodel

namespace objectmodel
{

// Specialization of Field::getValueTypeString() method to display smaller 
// The next line hides all those methods from the doxygen documentation
/// \cond TEMPLATE_OVERRIDES

template<>
inline std::string FieldBase::typeName< defaulttype::Quat >(const defaulttype::Quat *) { return "Quat"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Quat > >(const std::vector< defaulttype::Quat > *) { return "vector<Quat>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Quat > >(const helper::vector< defaulttype::Quat > *) { return "vector<Quat>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Quatf >(const defaulttype::Quatf *) { return "Quatf"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Quatf > >(const std::vector< defaulttype::Quatf > *) { return "vector<Quatf>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Quatf > >(const helper::vector< defaulttype::Quatf > *) { return "vector<Quatf>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid2dTypes::Coord >(const defaulttype::Rigid2dTypes::Coord *) { return "Rigid2dTypes::Coord"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid2dTypes::Coord > >(const std::vector< defaulttype::Rigid2dTypes::Coord > *) { return "vector<Rigid2dTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid2dTypes::Coord > >(const helper::vector< defaulttype::Rigid2dTypes::Coord > *) { return "vector<Rigid2dTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid2dTypes::Deriv >(const defaulttype::Rigid2dTypes::Deriv *) { return "Rigid2dTypes::Deriv"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid2dTypes::Deriv > >(const std::vector< defaulttype::Rigid2dTypes::Deriv > *) { return "vector<Rigid2dTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid2dTypes::Deriv > >(const helper::vector< defaulttype::Rigid2dTypes::Deriv > *) { return "vector<Rigid2dTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid2fTypes::Coord >(const defaulttype::Rigid2fTypes::Coord *) { return "Rigid2fTypes::Coord"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid2fTypes::Coord > >(const std::vector< defaulttype::Rigid2fTypes::Coord > *) { return "vector<Rigid2fTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid2fTypes::Coord > >(const helper::vector< defaulttype::Rigid2fTypes::Coord > *) { return "vector<Rigid2fTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid2fTypes::Deriv >(const defaulttype::Rigid2fTypes::Deriv *) { return "Rigid2fTypes::Deriv"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid2fTypes::Deriv > >(const std::vector< defaulttype::Rigid2fTypes::Deriv > *) { return "vector<Rigid2fTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid2fTypes::Deriv > >(const helper::vector< defaulttype::Rigid2fTypes::Deriv > *) { return "vector<Rigid2fTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid3dTypes::Coord >(const defaulttype::Rigid3dTypes::Coord *) { return "Rigid3dTypes::Coord"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid3dTypes::Coord > >(const std::vector< defaulttype::Rigid3dTypes::Coord > *) { return "vector<Rigid3dTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid3dTypes::Coord > >(const helper::vector< defaulttype::Rigid3dTypes::Coord > *) { return "vector<Rigid3dTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid3dTypes::Deriv >(const defaulttype::Rigid3dTypes::Deriv *) { return "Rigid3dTypes::Deriv"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid3dTypes::Deriv > >(const std::vector< defaulttype::Rigid3dTypes::Deriv > *) { return "vector<Rigid3dTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid3dTypes::Deriv > >(const helper::vector< defaulttype::Rigid3dTypes::Deriv > *) { return "vector<Rigid3dTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid3fTypes::Coord >(const defaulttype::Rigid3fTypes::Coord *) { return "Rigid3fTypes::Coord"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid3fTypes::Coord > >(const std::vector< defaulttype::Rigid3fTypes::Coord > *) { return "vector<Rigid3fTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid3fTypes::Coord > >(const helper::vector< defaulttype::Rigid3fTypes::Coord > *) { return "vector<Rigid3fTypes::Coord>"; }

template<>
inline std::string FieldBase::typeName< defaulttype::Rigid3fTypes::Deriv >(const defaulttype::Rigid3fTypes::Deriv *) { return "Rigid3fTypes::Deriv"; }

template<>
inline std::string FieldBase::typeName< std::vector< defaulttype::Rigid3fTypes::Deriv > >(const std::vector< defaulttype::Rigid3fTypes::Deriv > *) { return "vector<Rigid3fTypes::Deriv>"; }

template<>
inline std::string FieldBase::typeName< helper::vector< defaulttype::Rigid3fTypes::Deriv > >(const helper::vector< defaulttype::Rigid3fTypes::Deriv > *) { return "vector<Rigid3fTypes::Deriv>"; }

/// \endcond

} // namespace objectmodel

} // namespace core

} // namespace sofa


#endif
