//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Simple Spring Mass System
// -- vector lib
//
// Primary Author: James F. O'Brien (obrienj@cc.gatech.edu)
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Copyright (c) 2003-2005, Regents of the University of California.  All
// rights reserved.
//
// This software is part of the Berkeley Fluid Animation & Simulation
// Toolkit.  The name "Berkeley Fluid Animation & Simulation Toolkit" is
// a trademark of the Regents of the University of California.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//   Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//
//  Redistributions in binary form as an executable program, or part of
//  an executable program must, when run for the first time by a given
//  user, prominently display the above copyright notice, this list of
//  conditions and the following disclaimer.
//
//  Neither the name of the University of California, Berkeley nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written
//  permission.
//
//  ** Animations, still images, or other works created using this
//  ** software must clearly indicate in the list of credits that this
//  ** software was used.  The software shall be referred to as the
//  ** Berkeley Fluid Animation & Simulation Toolkit.  If the software
//  ** is included, either in whole or in part, within another software
//  ** package, credit must still be given to the Berkeley Fluid
//  ** Animation & Simulation Toolkit.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------

#ifndef BfastVECTOR_is_defined
#define BfastVECTOR_is_defined

//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Classes from this header:
namespace SLC {

class BfastVector2;
class BfastVector3;

}

#include "bfast.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

//-------------------------------------------------------------------
//-------------------------------------------------------------------
namespace SLC {

class BfastVector3 {
  // Class to store a three dimensional vector.
  // Expected uses include point in R3, RGB color, etc.
public:

  //----------------------------------------------
  // Constructors

  inline BfastVector3();
  inline BfastVector3(BfastReal d);
  inline BfastVector3(BfastReal d0,BfastReal d1,BfastReal d2);

  inline BfastVector3(const BfastVector3 &da);
  inline BfastVector3(const BfastReal    *da);  
  // da should point to a BfastReal[3] that will be copied


  //----------------------------------------------
  // Index operators

  inline BfastReal &operator[](unsigned int i)      ;
  inline BfastReal  operator[](unsigned int i) const;

  inline BfastReal &operator()(unsigned int i)      ;
  inline BfastReal  operator()(unsigned int i) const;


  //----------------------------------------------
  // Assignment and set

  inline BfastVector3 &set(BfastReal d);
  inline BfastVector3 &set(BfastReal d0, BfastReal d1, BfastReal d2);

  inline BfastVector3 &set(const BfastVector3 &da);
  inline BfastVector3 &set(const BfastReal    *da);  
  // da should point to a BfastReal[3] that will be copied

  inline BfastVector3 &operator=(BfastReal d);
  inline BfastVector3 &operator=(const BfastVector3 &da);
  inline BfastVector3 &operator=(const BfastReal    *da);  
  

  //----------------------------------------------
  // Comparison operators

  inline int operator==(const BfastVector3 &da) const;
  inline int operator!=(const BfastVector3 &da) const;

  inline int operator==(BfastReal d) const;
  inline int operator!=(BfastReal d) const;


  //----------------------------------------------
  // In place arithmetic

  inline BfastVector3 &operator+=(BfastReal d);
  inline BfastVector3 &operator-=(BfastReal d);
  inline BfastVector3 &operator*=(BfastReal d);
  inline BfastVector3 &operator/=(BfastReal d);

  inline BfastVector3 &operator+=(const BfastVector3 &da);
  inline BfastVector3 &operator-=(const BfastVector3 &da);
  inline BfastVector3 &operator*=(const BfastVector3 &da);
  inline BfastVector3 &operator/=(const BfastVector3 &da);
  // Componentwise operations

  inline BfastVector3 &maxSet(const BfastVector3 &da);
  inline BfastVector3 &minSet(const BfastVector3 &da);
  // Sets data[i] = max(data[i],da[i]) or min


  //----------------------------------------------
  // Static methods

  inline static unsigned int cycleAxis(unsigned int axis, int direction);

  //----------------------------------------------
  // Define Components

  enum Index { X = 0 , Y = 1 , Z = 2 , 
	       U = 0 , V = 1 , W = 2 ,
	       R = 0 , G = 1 , B = 2 };

public:
  //----------------------------------------------
  // Public data members

  BfastReal data[3];

  //----------------------------------------------
};


//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Operators for class BfastVector3

inline BfastVector3 operator-(const BfastVector3 &a);

inline BfastVector3 operator+(const BfastVector3 &a,const BfastVector3 &b);
inline BfastVector3 operator-(const BfastVector3 &a,const BfastVector3 &b);
inline BfastVector3 operator*(const BfastVector3 &a,const BfastVector3 &b);
inline BfastVector3 operator/(const BfastVector3 &a,const BfastVector3 &b);

inline BfastVector3 operator+(const BfastVector3 &a, BfastReal b);
inline BfastVector3 operator-(const BfastVector3 &a, BfastReal b);
inline BfastVector3 operator*(const BfastVector3 &a, BfastReal b);
inline BfastVector3 operator/(const BfastVector3 &a, BfastReal b);

inline BfastVector3 operator+(BfastReal a, const BfastVector3 &b);
inline BfastVector3 operator-(BfastReal a, const BfastVector3 &b);
inline BfastVector3 operator*(BfastReal a, const BfastVector3 &b);
inline BfastVector3 operator/(BfastReal a, const BfastVector3 &b);

std::istream &operator>>(std::istream &strm,      BfastVector3 &v);
std::ostream &operator<<(std::ostream &strm,const BfastVector3 &v);

//-------------------------------------------------------------------
// Norm type functions for BfastVector3

inline BfastReal   l1Norm(const BfastVector3 &a);
inline BfastReal   l2Norm(const BfastVector3 &a);
inline BfastReal lInfNorm(const BfastVector3 &a);
// Computes the l1, l2 or lInfinity norm of a

inline BfastReal    mag(const BfastVector3 &a);
inline BfastReal sqrMag(const BfastVector3 &a);
// mag is the l2Norm or magnitude of the vector
// sqrMag is mag^2, which is faster to compute

inline void normalize(BfastVector3 &a);
// SETS a = a/mag(a)


//-------------------------------------------------------------------
// Other functions for BfastVector3

inline unsigned int dominantAxis(const BfastVector3 &v);
inline unsigned int subinantAxis(const BfastVector3 &v);
inline unsigned int midinantAxis(const BfastVector3 &v);
// Returns the index of the component with the largest,
// smallest or middle value.  Note: subinantAxis and
// midinantAxis are nore really words, I made them up.
// If multiple comonents have the same value, then the
// results are not unique.

inline BfastReal      dot(const BfastVector3 &a,const BfastVector3 &b);
inline BfastVector3 cross(const BfastVector3 &a,const BfastVector3 &b);
// Compute the dot and cros product of a and b.

inline BfastReal box(const BfastVector3 &a,const BfastVector3 &b,const BfastVector3 &c);
// Compute the box (aka tripple) product of a, b, and d.

inline BfastVector3 abs(const BfastVector3 &a);
// returns a vector with r[i] = abs(a[i])

inline BfastReal sum(const BfastVector3 &a);
// return a[0]+a[1]+a[2]

inline BfastReal max(const BfastVector3 &a);
inline BfastReal min(const BfastVector3 &a);
// Returns the max or min component of a.

inline BfastVector3 max(const BfastVector3 &a,const BfastVector3 &b);
inline BfastVector3 min(const BfastVector3 &a,const BfastVector3 &b);
// Computes a NEW vector by taking the max component in the
// x,y,z direction from a and b.  ie: r[i] = max(a[i],b[i])
// Note: signed values are used.


//-------------------------------------------------------------------
//-------------------------------------------------------------------


class BfastVector2 {
  // Class to store a two dimensional vector.
  // Expected uses include point in R2, etc.
public:

  //----------------------------------------------
  // Constructors

  inline BfastVector2();
  inline BfastVector2(BfastReal d);
  inline BfastVector2(BfastReal d0,BfastReal d1);

  inline BfastVector2(const BfastVector2 &da);
  inline BfastVector2(const BfastReal    *da);  
  // da should point to a BfastReal[2] that will be copied


  //----------------------------------------------
  // Index operators

  inline BfastReal &operator[](unsigned int i)      ;
  inline BfastReal  operator[](unsigned int i) const;

  inline BfastReal &operator()(unsigned int i)      ;
  inline BfastReal  operator()(unsigned int i) const;


  //----------------------------------------------
  // Assignment and set

  inline BfastVector2 &set(BfastReal d);
  inline BfastVector2 &set(BfastReal d0, BfastReal d1);

  inline BfastVector2 &set(const BfastVector2 &da);
  inline BfastVector2 &set(const BfastReal    *da);  
  // da should point to a BfastReal[3] that will be copied

  inline BfastVector2 &operator=(BfastReal d);
  inline BfastVector2 &operator=(const BfastVector2 &da);
  inline BfastVector2 &operator=(const BfastReal    *da);  
  
  //----------------------------------------------
  // Comparison operators

  inline int operator==(const BfastVector2 &da) const;
  inline int operator!=(const BfastVector2 &da) const;

  inline int operator==(BfastReal d) const;
  inline int operator!=(BfastReal d) const;


  //----------------------------------------------
  // In place arithmetic

  inline BfastVector2 &operator+=(BfastReal d);
  inline BfastVector2 &operator-=(BfastReal d);
  inline BfastVector2 &operator*=(BfastReal d);
  inline BfastVector2 &operator/=(BfastReal d);

  inline BfastVector2 &operator+=(const BfastVector2 &da);
  inline BfastVector2 &operator-=(const BfastVector2 &da);
  inline BfastVector2 &operator*=(const BfastVector2 &da);
  inline BfastVector2 &operator/=(const BfastVector2 &da);
  // Componentwise operations

  inline BfastVector2 &maxSet(const BfastVector2 &da);
  inline BfastVector2 &minSet(const BfastVector2 &da);
  // Sets data[i] = max(data[i],da[i]) or min


  //----------------------------------------------
  // Static methods

  inline static unsigned int cycleAxis(unsigned int axis, int direction);

  //----------------------------------------------
  // Define Components

  enum Index { X = 0 , Y = 1 ,
	       U = 0 , V = 1 };

public:
  //----------------------------------------------
  // Public data members

  BfastReal data[2];

  //----------------------------------------------
};


//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Operators for class BfastVector2

inline BfastVector2 operator-(const BfastVector2 &a);

inline BfastVector2 operator+(const BfastVector2 &a, const BfastVector2 &b);
inline BfastVector2 operator-(const BfastVector2 &a, const BfastVector2 &b);
inline BfastVector2 operator*(const BfastVector2 &a, const BfastVector2 &b);
inline BfastVector2 operator/(const BfastVector2 &a, const BfastVector2 &b);

inline BfastVector2 operator+(const BfastVector2 &a, BfastReal b);
inline BfastVector2 operator-(const BfastVector2 &a, BfastReal b);
inline BfastVector2 operator*(const BfastVector2 &a, BfastReal b);
inline BfastVector2 operator/(const BfastVector2 &a, BfastReal b);

inline BfastVector2 operator+(BfastReal a, const BfastVector2 &b);
inline BfastVector2 operator-(BfastReal a, const BfastVector2 &b);
inline BfastVector2 operator*(BfastReal a, const BfastVector2 &b);
inline BfastVector2 operator/(BfastReal a, const BfastVector2 &b);

std::istream &operator>>(std::istream &strm,      BfastVector2 &v);
std::ostream &operator<<(std::ostream &strm,const BfastVector2 &v);

//-------------------------------------------------------------------
// Norm type functions for BfastVector2

inline BfastReal   l1Norm(const BfastVector2 &a);
inline BfastReal   l2Norm(const BfastVector2 &a);
inline BfastReal lInfNorm(const BfastVector2 &a);
// Computes the l1, l2 or lInfinity norm of a

inline BfastReal    mag(const BfastVector2 &a);
inline BfastReal sqrMag(const BfastVector2 &a);
// mag is the l2Norm or magnitude of the vector
// sqrMag is mag^2, which is faster to compute

inline void normalize(BfastVector2 &a);
// SETS a = a/mag(a)


//-------------------------------------------------------------------
// Other functions for BfastVector2

inline unsigned int dominantAxis(const BfastVector2 &v);
inline unsigned int subinantAxis(const BfastVector2 &v);
// Returns the index of the component with the largest,
// smallest value.  Note: subinantAxis and is not really
// a word, I made it up.
// If multiple comonents have the same value, then the
// results are not unique.

inline BfastReal   dot(const BfastVector2 &a,const BfastVector2 &b);
inline BfastReal cross(const BfastVector2 &a,const BfastVector2 &b);
// Compute the dot and cros product of a and b.

inline BfastVector2 abs(const BfastVector2 &a);
// returns a vector with r[i] = abs(a[i])

inline BfastReal sum(const BfastVector2 &a);
// return a[0]+a[1]

inline BfastReal max(const BfastVector2 &a);
inline BfastReal min(const BfastVector2 &a);
// Returns the max or min component of a.

inline BfastVector2 max(const BfastVector2 &a,const BfastVector2 &b);
inline BfastVector2 min(const BfastVector2 &a,const BfastVector2 &b);
// Computes a NEW vector by taking the max component in the
// x,y,z direction from a and b.  ie: r[i] = max(a[i],b[i])
// Note: signed values are used.




//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Inline implementation below:::

#ifndef DB_CHECK
#ifdef DEBUG
#define DB_CHECK( C ) { if ( ! (C) ) { abort(); } }
#else
#define DB_CHECK( C ) { }
#endif
#endif

//-------------------------------------------------------------------
//-------------------------------------------------------------------

// Inline implementation of BfastVector3

inline BfastReal &BfastVector3::operator[](unsigned int i) {
  DB_CHECK(i<3);
  return data[i];
}
inline BfastReal &BfastVector3::operator()(unsigned int i) {
  DB_CHECK(i<3);
  return data[i];
}

inline BfastReal  BfastVector3::operator[](unsigned int i) const {
  DB_CHECK(i<3);
  return data[i];
}
  
inline BfastReal  BfastVector3::operator()(unsigned int i) const {
  DB_CHECK(i<3);
  return data[i];
}

//-------------------------------------------------------------------

inline BfastVector3::BfastVector3() {
  data[0] = data[1] = data[2] = 0.0;
}
inline BfastVector3::BfastVector3(BfastReal d) {
  data[0] = data[1] = data[2] = d;
}

inline BfastVector3::BfastVector3(BfastReal d0,BfastReal d1,BfastReal d2) {
  data[0] = d0;
  data[1] = d1;
  data[2] = d2;
}

inline BfastVector3::BfastVector3(const BfastVector3 &da) {
  data[0] = da[0];
  data[1] = da[1];
  data[2] = da[2];
}

inline BfastVector3::BfastVector3(const BfastReal *da) {
  data[0] = da[0];
  data[1] = da[1];
  data[2] = da[2];
}
  
//-------------------------------------------------------------------

inline BfastVector3 &BfastVector3::set(BfastReal d) {
  data[0] = d;
  data[1] = d;
  data[2] = d;
  return (*this);
}
  
inline BfastVector3 &BfastVector3::set(BfastReal d0, BfastReal d1, BfastReal d2) {
  data[0] = d0;
  data[1] = d1;
  data[2] = d2;
  return (*this);
}

inline BfastVector3 &BfastVector3::set(const BfastVector3 &da) {
  data[0] = da[0];
  data[1] = da[1];
  data[2] = da[2];
  return (*this);
}
  
inline BfastVector3 &BfastVector3::set(const BfastReal *da) {
  data[0] = da[0];
  data[1] = da[1];
  data[2] = da[2];
  return (*this);
}
  
//-------------------------------------------------------------------

inline BfastVector3 &BfastVector3::operator=(BfastReal d) {
  return set(d);
}
 
inline BfastVector3 &BfastVector3::operator=(const BfastVector3 &da) {
  return set(da);
}

inline BfastVector3 &BfastVector3::operator=(const BfastReal *da) {
  return set(da);
}
  
//-------------------------------------------------------------------

inline int BfastVector3::operator==(const BfastVector3 &da) const {
  return ((data[0] == da[0]) &&
	  (data[1] == da[1]) &&
	  (data[2] == da[2]));
}

inline int BfastVector3::operator!=(const BfastVector3 &da) const {
  return ((data[0] != da[0]) ||
	  (data[1] != da[1]) ||
	  (data[2] != da[2]));
}
  
inline int BfastVector3::operator==(BfastReal d) const {
  return ((data[0] == d) &&
	  (data[1] == d) &&
	  (data[2] == d));
}

inline int BfastVector3::operator!=(BfastReal d) const {
  return ((data[0] != d) ||
	  (data[1] != d) ||
	  (data[2] != d));
}

//-------------------------------------------------------------------

inline BfastVector3 &BfastVector3::operator+=(BfastReal d) {
  data[0] += d;
  data[1] += d;
  data[2] += d;
  return (*this);
}
    
inline BfastVector3 &BfastVector3::operator-=(BfastReal d) {
  data[0] -= d;
  data[1] -= d;
  data[2] -= d;
  return (*this);
}

inline BfastVector3 &BfastVector3::operator*=(BfastReal d) {
  data[0] *= d;
  data[1] *= d;
  data[2] *= d;
  return (*this);
}

inline BfastVector3 &BfastVector3::operator/=(BfastReal d) {
  data[0] /= d;
  data[1] /= d;
  data[2] /= d;
  return (*this);
}

//-------------------------------------------------------------------

inline BfastVector3 &BfastVector3::operator+=(const BfastVector3 &da) {
  data[0] += da[0];
  data[1] += da[1];
  data[2] += da[2];
  return (*this);
}

inline BfastVector3 &BfastVector3::operator-=(const BfastVector3 &da) {
  data[0] -= da[0];
  data[1] -= da[1];
  data[2] -= da[2];
  return (*this);
}

inline BfastVector3 &BfastVector3::operator*=(const BfastVector3 &da) {
  data[0] *= da[0];
  data[1] *= da[1];
  data[2] *= da[2];
  return (*this);
}

inline BfastVector3 &BfastVector3::operator/=(const BfastVector3 &da) {
  data[0] /= da[0];
  data[1] /= da[1];
  data[2] /= da[2];
  return (*this);
}

//-------------------------------------------------------------------

inline BfastVector3 &BfastVector3::maxSet(const BfastVector3 &da) {
  if (da[0] > data[0]) data[0] = da[0];
  if (da[1] > data[1]) data[1] = da[1];
  if (da[2] > data[2]) data[2] = da[2];
  return (*this);
}

inline BfastVector3 &BfastVector3::minSet(const BfastVector3 &da) {
  if (da[0] < data[0]) data[0] = da[0];
  if (da[1] < data[1]) data[1] = da[1];
  if (da[2] < data[2]) data[2] = da[2];
  return (*this);
}
  
//-------------------------------------------------------------------

inline unsigned int BfastVector3::cycleAxis(unsigned int axis, int direction) {
  switch (axis+direction) {
  case 0: case 3: case 6: return 0;
  case 1: case 4: case 7: return 1;
  case 2: case 5: case 8: return 2;
  default: return (axis+direction)%3;
  }
}

//-------------------------------------------------------------------

inline BfastVector3 operator-(const BfastVector3 &a) {
  return BfastVector3(-a[0],-a[1],-a[2]);
}

//-------------------------------------------------------------------

inline BfastVector3 operator+(const BfastVector3 &a,const BfastVector3 &b) {
  return BfastVector3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline BfastVector3 operator-(const BfastVector3 &a,const BfastVector3 &b) {
  return BfastVector3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline BfastVector3 operator*(const BfastVector3 &a,const BfastVector3 &b){
  return BfastVector3(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

inline BfastVector3 operator/(const BfastVector3 &a,const BfastVector3 &b){
  return BfastVector3(a[0] / b[0], a[1] / b[1], a[2] / b[2]);
}

//-------------------------------------------------------------------

inline BfastVector3 operator+(const BfastVector3 &a,BfastReal b){
  return BfastVector3(a[0] + b, a[1] + b, a[2] + b);
}

inline BfastVector3 operator-(const BfastVector3 &a,BfastReal b){
  return BfastVector3(a[0] - b, a[1] - b, a[2] - b);
}

inline BfastVector3 operator*(const BfastVector3 &a,BfastReal b){
  return BfastVector3(a[0] * b, a[1] * b, a[2] * b);
}

inline BfastVector3 operator/(const BfastVector3 &a,BfastReal b){
  return BfastVector3(a[0] / b, a[1] / b, a[2] / b);
}

//-------------------------------------------------------------------

inline BfastVector3 operator+(BfastReal a,const BfastVector3 &b){
  return BfastVector3(a + b[0], a + b[1], a + b[2]);
}

inline BfastVector3 operator-(BfastReal a,const BfastVector3 &b){
  return BfastVector3(a - b[0], a - b[1], a - b[2]);
}

inline BfastVector3 operator*(BfastReal a,const BfastVector3 &b){
  return BfastVector3(a * b[0], a * b[1], a * b[2]);
}

inline BfastVector3 operator/(BfastReal a,const BfastVector3 &b){
  return BfastVector3(a / b[0], a / b[1], a / b[2]);
}

//-------------------------------------------------------------------

inline BfastReal l1Norm(const BfastVector3 &a) {
  return (((a[0]>0)?a[0]:-a[0])+
	  ((a[1]>0)?a[1]:-a[1])+
	  ((a[2]>0)?a[2]:-a[2]));
}
  
inline BfastReal l2Norm(const BfastVector3 &a) {
  return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
  
inline BfastReal lInfNorm(const BfastVector3 &a) {
  return max(abs(a));
}

inline BfastReal mag(const BfastVector3 &a) {
  return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

inline BfastReal sqrMag(const BfastVector3 &a) {
  return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

inline void normalize(BfastVector3 &a) {
  register BfastReal m = mag(a);
  if (m != 0) a /= m;
}

//-------------------------------------------------------------------

inline unsigned int dominantAxis(const BfastVector3 &v) {
  register BfastReal x,y,z;
  if (v[0]>0) x = v[0]; else x = -v[0];
  if (v[1]>0) y = v[1]; else y = -v[1];
  if (v[2]>0) z = v[2]; else z = -v[2];
  return ( x > y ) ? (( x > z ) ? 0 : 2) : (( y > z ) ? 1 : 2 );
}

inline unsigned int subinantAxis(const BfastVector3 &v) {
  register BfastReal x,y,z;
  if (v[0]>0) x = v[0]; else x = -v[0];
  if (v[1]>0) y = v[1]; else y = -v[1];
  if (v[2]>0) z = v[2]; else z = -v[2];
  return ( x < y ) ? (( x < z ) ? 0 : 2) : (( y < z ) ? 1 : 2 );
}

inline unsigned int midinantAxis(const BfastVector3 &v) {
  register BfastReal x,y,z;
  if (v[0]>0) x = v[0]; else x = -v[0];
  if (v[1]>0) y = v[1]; else y = -v[1];
  if (v[2]>0) z = v[2]; else z = -v[2];
  register unsigned int d = ( x > y ) ? (( x > z ) ? 0 : 2) : (( y > z ) ? 1 : 2 );
  register unsigned int s = ( x < y ) ? (( x < z ) ? 0 : 2) : (( y < z ) ? 1 : 2 );
  register unsigned int m;
  if (d==0) {
    if (s!= 1) m = 1; else m = 2;
  }else if (d==1) {
    if (s!= 0) m = 0; else m = 2;
  }else if (d==2) {
    if (s!= 0) m = 0; else m = 1;
  }
  return m;
}

//-------------------------------------------------------------------

inline BfastReal dot(const BfastVector3 &a,const BfastVector3 &b) {
  return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}
  
inline BfastVector3 cross(const BfastVector3 &a,const BfastVector3 &b) {
  return BfastVector3(a[1] * b[2] - b[1] * a[2],
		   a[2] * b[0] - b[2] * a[0],
		   a[0] * b[1] - b[0] * a[1]);
}

//-------------------------------------------------------------------

inline BfastReal box(const BfastVector3 &a,const BfastVector3 &b,const BfastVector3 &c) {
  return dot(cross(a,b),c);
}

//-------------------------------------------------------------------

inline BfastVector3 abs(const BfastVector3 &a) {
  return  BfastVector3(((a[0]>0)?a[0]:-a[0]),
		    ((a[1]>0)?a[1]:-a[1]),
		    ((a[2]>0)?a[2]:-a[2]));
} 

inline BfastReal sum(const BfastVector3 &a) {
  return a[0]+a[1]+a[2];
}

//-------------------------------------------------------------------

inline BfastReal max(const BfastVector3 &a) {
  return ((a[0]>a[1])?((a[0]>a[2])?a[0]:a[2]):(a[1]>a[2])?a[1]:a[2]);
}	  

inline BfastReal min(const BfastVector3 &a) {
  return ((a[0]<a[1])?((a[0]<a[2])?a[0]:a[2]):(a[1]<a[2])?a[1]:a[2]);
}	  

//-------------------------------------------------------------------

inline BfastVector3 max(const BfastVector3 &a,const BfastVector3 &b) {
  return BfastVector3((a[0]>b[0])?a[0]:b[0],
		   (a[1]>b[1])?a[1]:b[1],
		   (a[2]>b[2])?a[2]:b[2]);
}
 
inline BfastVector3 min(const BfastVector3 &a,const BfastVector3 &b) {
  return BfastVector3((a[0]<b[0])?a[0]:b[0],
		   (a[1]<b[1])?a[1]:b[1],
		   (a[2]<b[2])?a[2]:b[2]);
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------

// Inline implementation of BfastVector2

inline BfastReal &BfastVector2::operator[](unsigned int i) {
  DB_CHECK(i<2);
  return data[i];
}
inline BfastReal &BfastVector2::operator()(unsigned int i) {
  DB_CHECK(i<2);
  return data[i];
}

inline BfastReal  BfastVector2::operator[](unsigned int i) const {
  DB_CHECK(i<2);
  return data[i];
}
  
inline BfastReal  BfastVector2::operator()(unsigned int i) const {
  DB_CHECK(i<2);
  return data[i];
}

//-------------------------------------------------------------------

inline BfastVector2::BfastVector2() {
  data[0] = data[1] = 0.0;
}
inline BfastVector2::BfastVector2(BfastReal d) {
  data[0] = data[1] = d;
}

inline BfastVector2::BfastVector2(BfastReal d0,BfastReal d1) {
  data[0] = d0;
  data[1] = d1;
}

inline BfastVector2::BfastVector2(const BfastVector2 &da) {
  data[0] = da[0];
  data[1] = da[1];
}

inline BfastVector2::BfastVector2(const BfastReal *da) {
  data[0] = da[0];
  data[1] = da[1];
}
  
//-------------------------------------------------------------------

inline BfastVector2 &BfastVector2::set(BfastReal d) {
  data[0] = d;
  data[1] = d;
  return (*this);
}
  
inline BfastVector2 &BfastVector2::set(BfastReal d0, BfastReal d1) {
  data[0] = d0;
  data[1] = d1;
  return (*this);
}

inline BfastVector2 &BfastVector2::set(const BfastVector2 &da) {
  data[0] = da[0];
  data[1] = da[1];
  return (*this);
}
  
inline BfastVector2 &BfastVector2::set(const BfastReal *da) {
  data[0] = da[0];
  data[1] = da[1];
  return (*this);
}
  
//-------------------------------------------------------------------

inline BfastVector2 &BfastVector2::operator=(BfastReal d) {
  return set(d);
}
 
inline BfastVector2 &BfastVector2::operator=(const BfastVector2 &da) {
  return set(da);
}

inline BfastVector2 &BfastVector2::operator=(const BfastReal *da) {
  return set(da);
}
  
//-------------------------------------------------------------------

inline int BfastVector2::operator==(const BfastVector2 &da) const {
  return ((data[0] == da[0]) &&
	  (data[1] == da[1]));
}

inline int BfastVector2::operator!=(const BfastVector2 &da) const {
  return ((data[0] != da[0]) ||
	  (data[1] != da[1]));
}
  
inline int BfastVector2::operator==(BfastReal d) const {
  return ((data[0] == d) &&
	  (data[1] == d));
}

inline int BfastVector2::operator!=(BfastReal d) const {
  return ((data[0] != d) ||
	  (data[1] != d));
}

//-------------------------------------------------------------------

inline BfastVector2 &BfastVector2::operator+=(BfastReal d) {
  data[0] += d;
  data[1] += d;
  return (*this);
}
    
inline BfastVector2 &BfastVector2::operator-=(BfastReal d) {
  data[0] -= d;
  data[1] -= d;
  return (*this);
}

inline BfastVector2 &BfastVector2::operator*=(BfastReal d) {
  data[0] *= d;
  data[1] *= d;
  return (*this);
}

inline BfastVector2 &BfastVector2::operator/=(BfastReal d) {
  data[0] /= d;
  data[1] /= d;
  return (*this);
}

//-------------------------------------------------------------------

inline BfastVector2 &BfastVector2::operator+=(const BfastVector2 &da) {
  data[0] += da[0];
  data[1] += da[1];
  return (*this);
}

inline BfastVector2 &BfastVector2::operator-=(const BfastVector2 &da) {
  data[0] -= da[0];
  data[1] -= da[1];
  return (*this);
}

inline BfastVector2 &BfastVector2::operator*=(const BfastVector2 &da) {
  data[0] *= da[0];
  data[1] *= da[1];
  return (*this);
}

inline BfastVector2 &BfastVector2::operator/=(const BfastVector2 &da) {
  data[0] /= da[0];
  data[1] /= da[1];
  return (*this);
}

//-------------------------------------------------------------------

inline BfastVector2 &BfastVector2::maxSet(const BfastVector2 &da) {
  if (da[0] > data[0]) data[0] = da[0];
  if (da[1] > data[1]) data[1] = da[1];
  return (*this);
}

inline BfastVector2 &BfastVector2::minSet(const BfastVector2 &da) {
  if (da[0] < data[0]) data[0] = da[0];
  if (da[1] < data[1]) data[1] = da[1];
  return (*this);
}
  
//-------------------------------------------------------------------

inline unsigned int BfastVector2::cycleAxis(unsigned int axis, int direction) {
  switch (axis+direction) {
  case 0: case 2: case 4: return 0;
  case 1: case 3: case 5: return 1;
  default: return (axis+direction)%2;
  }
}

//-------------------------------------------------------------------

inline BfastVector2 operator-(const BfastVector2 &a) {
  return BfastVector2(-a[0],-a[1]);
}

//-------------------------------------------------------------------

inline BfastVector2 operator+(const BfastVector2 &a,const BfastVector2 &b) {
  return BfastVector2(a[0] + b[0], a[1] + b[1]);
}

inline BfastVector2 operator-(const BfastVector2 &a,const BfastVector2 &b) {
  return BfastVector2(a[0] - b[0], a[1] - b[1]);
}

inline BfastVector2 operator*(const BfastVector2 &a,const BfastVector2 &b){
  return BfastVector2(a[0] * b[0], a[1] * b[1]);
}

inline BfastVector2 operator/(const BfastVector2 &a,const BfastVector2 &b){
  return BfastVector2(a[0] / b[0], a[1] / b[1]);
}

//-------------------------------------------------------------------

inline BfastVector2 operator+(const BfastVector2 &a,BfastReal b){
  return BfastVector2(a[0] + b, a[1] + b);
}

inline BfastVector2 operator-(const BfastVector2 &a,BfastReal b){
  return BfastVector2(a[0] - b, a[1] - b);
}

inline BfastVector2 operator*(const BfastVector2 &a,BfastReal b){
  return BfastVector2(a[0] * b, a[1] * b);
}

inline BfastVector2 operator/(const BfastVector2 &a,BfastReal b){
  return BfastVector2(a[0] / b, a[1] / b);
}

//-------------------------------------------------------------------

inline BfastVector2 operator+(BfastReal a,const BfastVector2 &b){
  return BfastVector2(a + b[0], a + b[1]);
}

inline BfastVector2 operator-(BfastReal a,const BfastVector2 &b){
  return BfastVector2(a - b[0], a - b[1]);
}

inline BfastVector2 operator*(BfastReal a,const BfastVector2 &b){
  return BfastVector2(a * b[0], a * b[1]);
}

inline BfastVector2 operator/(BfastReal a,const BfastVector2 &b){
  return BfastVector2(a / b[0], a / b[1]);
}

//-------------------------------------------------------------------

inline BfastReal l1Norm(const BfastVector2 &a) {
  return (((a[0]>0)?a[0]:-a[0])+
	  ((a[1]>0)?a[1]:-a[1]));
}
  
inline BfastReal l2Norm(const BfastVector2 &a) {
  return sqrt(a[0] * a[0] + a[1] * a[1]);
}
  
inline BfastReal lInfNorm(const BfastVector2 &a) {
  return max(abs(a));
}

inline BfastReal mag(const BfastVector2 &a) {
  return sqrt(a[0] * a[0] + a[1] * a[1]);
}

inline BfastReal sqrMag(const BfastVector2 &a) {
  return (a[0] * a[0] + a[1] * a[1]);
}

inline void normalize(BfastVector2 &a) {
  register BfastReal m = mag(a);
  if (m != 0) a /= m;
}

//-------------------------------------------------------------------

inline unsigned int dominantAxis(const BfastVector2 &v) {
  register BfastReal x,y;
  if (v[0]>0) x = v[0]; else x = -v[0];
  if (v[1]>0) y = v[1]; else y = -v[1];
  return ( x > y ) ? 0 : 1;
}

inline unsigned int subinantAxis(const BfastVector2 &v) {
  register BfastReal x,y;
  if (v[0]>0) x = v[0]; else x = -v[0];
  if (v[1]>0) y = v[1]; else y = -v[1];
  return ( x < y ) ? 0 : 1;
}

//-------------------------------------------------------------------

inline BfastReal dot(const BfastVector2 &a,const BfastVector2 &b) {
  return (a[0] * b[0] + a[1] * b[1]);
}
  
inline BfastReal cross(const BfastVector2 &a,const BfastVector2 &b) {
  return (a[0] * b[1] - b[0] * a[1]);
}

//-------------------------------------------------------------------

inline BfastVector2 abs(const BfastVector2 &a) {
  return  BfastVector2(((a[0]>0)?a[0]:-a[0]),
		    ((a[1]>0)?a[1]:-a[1]));
} 

inline BfastReal sum(const BfastVector2 &a) {
  return a[0]+a[1];
}

//-------------------------------------------------------------------

inline BfastReal max(const BfastVector2 &a) {
  return ((a[0]>a[1])? a[0] : a[1]);
}	  

inline BfastReal min(const BfastVector2 &a) {
  return ((a[0]<a[1])? a[0] : a[1]);
}	  

//-------------------------------------------------------------------

inline BfastVector2 max(const BfastVector2 &a,const BfastVector2 &b) {
  return BfastVector2((a[0]>b[0])?a[0]:b[0],
		   (a[1]>b[1])?a[1]:b[1]);
}
 
inline BfastVector2 min(const BfastVector2 &a,const BfastVector2 &b) {
  return BfastVector2((a[0]<b[0])?a[0]:b[0],
		   (a[1]<b[1])?a[1]:b[1]);
}



//-------------------------------------------------------------------
//-------------------------------------------------------------------
}

#endif

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: bfastVector.H,v $
// Revision 1.3  2006/06/16 21:07:14  adamb
// new version
//
// Revision 1.2  2006/01/15 01:43:52  adamb
// added rebuildTree
//
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.1  2005/09/01 21:06:46  adamb
// SmVector->BfastVector
//
// Revision 1.1.1.1  2005/05/25 05:41:05  adamb
// Initial Revision
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
