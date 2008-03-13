//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Useful Stuff
//   -- Header File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
// 
// This file contains a bunch of useful functions, and classes.
// List of doubles, integers, vectors, tirangles, etc.
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

#ifndef bfastUtil_H
#define bfastUtil_H

#include "bfast.h"
#include <iostream>
#include "bfastVector.h"

#ifdef WIN32
#if _MSC_VER >= 1300
#include <hash_map>
#include <hash_set>
#if _MSC_VER >= 1400
#ifndef HASH_MAP
#define HASH_MAP stdext::hash_map
#endif
#ifndef HASH_SET
#define HASH_SET stdext::hash_set
#endif
#else
#ifndef HASH_MAP
#define HASH_MAP std::hash_map
#endif
#ifndef HASH_SET
#define HASH_SET std::hash_set
#endif
#endif
#else
#include <map>
#include <set>
#ifndef HASH_MAP
#define HASH_MAP std::map
#endif
#ifndef HASH_SET
#define HASH_SET std::set
#endif
#endif
#else
#include <ext/hash_map>
#include <ext/hash_set>
#ifndef HASH_MAP
#define HASH_MAP __gnu_cxx::hash_map
#endif
#ifndef HASH_SET
#define HASH_SET __gnu_cxx::hash_set
#endif
#endif


#ifdef INVENTOR
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#endif

namespace SLC
{

class BfastIndexList {
 public:
  BfastIndexList();
  BfastIndexList(const BfastIndexList &that);
  ~BfastIndexList() { delete [] _values;};

  BfastIndexList &operator=(const BfastIndexList &that);

  inline int &index(int i) {return _values[i];};
  inline const int &index(int i) const {return _values[i];};
  inline int &operator[](int i) {return _values[i];};
  inline const int &operator[](int i) const {return _values[i];};

  inline unsigned int getNum() const {return _numValues;};
  void setNum(unsigned int num, int copyOld = 0, int exactSize = 1);
  void add(int value);
  void add(BfastIndexList &list);
  void add(int value, int index);
  void del(int index);
  void delValue(int value);
  void swap(int index1, int index2);
  void clear();
  int contains(int value);
  inline int pop();
  int find(int value);

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numValues;
  int _numMalloc;
  int *_values;
};
std::istream &operator>>(std::istream &strm, BfastIndexList &p);
std::ostream &operator<<(std::ostream &strm, const BfastIndexList &p);

///////////////////////////////////////////////////////////////////
  
class BfastRealList {
 public:
  BfastRealList();
  BfastRealList(const BfastRealList &that);
  ~BfastRealList() { if (_values) delete [] _values;};

  BfastRealList &operator=(const BfastRealList &that);

  inline BfastReal &index(int i) {return _values[i];};
  inline const BfastReal &index(int i) const {return _values[i];};
  inline BfastReal &operator[](int i) {return _values[i];};
  inline const BfastReal &operator[](int i) const {return _values[i];};

  inline unsigned int numValues() const {return _numValues;};
  void setNumValues(unsigned int num, int copyOld = 0, int exactSize = 1);

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numValues;
  int _numMalloc;
  BfastReal *_values;
};
std::istream &operator>>(std::istream &strm, BfastRealList &p);
std::ostream &operator<<(std::ostream &strm, const BfastRealList &p);
  
/////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////

class BfastTri {
public:
	int a, b, c;
  BfastTri &operator=(const BfastTri &that) {a=that.a; b=that.b; c=that.c; return *this;};

	void read(std::ifstream &in);
	void write(std::ofstream &out);
};
std::istream &operator>>(std::istream &strm, BfastTri &t);
std::ostream &operator<<(std::ostream &strm, const BfastTri &t);

class BfastTriList {
 public:
  BfastTriList();
  BfastTriList(const BfastTriList &that);
  ~BfastTriList() { delete [] _triangles;};

  BfastTriList &operator=(const BfastTriList &that);

  inline BfastTri &index(int i) {return _triangles[i];};
  inline const BfastTri &index(int i) const {return _triangles[i];};
  inline BfastTri &operator[](int i) {return _triangles[i];};
  inline const BfastTri &operator[](int i) const {return _triangles[i];};

  inline unsigned int numTriangles() const {return _numTriangles;};
  void setNumTriangles(unsigned int num, int copyOld = 0, int exactSize = 1);
	
	int add(int a, int b, int c);

#ifdef INVENTOR
	SoIndexedFaceSet *faceSet();
#endif

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numTriangles;
  int _numMalloc;
  BfastTri *_triangles;
};
std::istream &operator>>(std::istream &strm, BfastTriList &p);
std::ostream &operator<<(std::ostream &strm, const BfastTriList &p);
  
/////////////////////////////////////////////////////////////////

class BfastVecList {
 public:
  BfastVecList();
  BfastVecList(const BfastVecList &that);
  ~BfastVecList() { delete [] _vecs;};

  BfastVecList &operator=(const BfastVecList &that);

  inline BfastVector3 &index(int i) {return _vecs[i];};
  inline const BfastVector3 &index(int i) const {return _vecs[i];};
  inline BfastVector3 &operator[](int i) {return _vecs[i];};
  inline const BfastVector3 &operator[](int i) const {return _vecs[i];};

  inline unsigned int numVecs() const {return _numVecs;};
  void setNumVecs(unsigned int num, int copyOld = 0, int exactSize = 1);
	void clear();

#ifdef INVENTOR
	SoCoordinate3 *coord();
#endif

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numVecs;
  int _numMalloc;
  BfastVector3 *_vecs;
};
std::istream &operator>>(std::istream &strm, BfastVecList &p);
std::ostream &operator<<(std::ostream &strm, const BfastVecList &p);
  
/////////////////////////////////////////////////////////////////

class BfastInt3 {
public:
	inline BfastInt3() { data[0] = data[1] = data[2] = 0; };
	inline BfastInt3(int i) { data[0] = data[1] = data[2] = i; };
	inline BfastInt3(int i1, int i2, int i3) { data[0] = i1; data[1] = i2; data[2] = i3; };
	inline BfastInt3(const BfastInt3 &da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; };
	inline BfastInt3(const int *da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; };

	inline int &operator[](unsigned int i) { return data[i]; };
	inline int  operator[](unsigned int i) const { return data[i]; };

	inline int &operator()(unsigned int i) { return data[i]; };
	inline int  operator()(unsigned int i) const { return data[i]; };

	inline BfastInt3 &set(int i) { data[0] = data[1] = data[2] = i; return *this; };
	inline BfastInt3 &set(int i1, int i2, int i3) { data[0] = i1; data[1] = i2; data[2] = i3; return *this; };
	
	inline BfastInt3 &set(const BfastInt3 &da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; return *this; };
	inline BfastInt3 &set(const int *da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; return *this; };
	
	inline BfastInt3 &operator=(int i) { data[0] = data[1] = data[2] = i; return *this; };
	inline BfastInt3 &operator=(const BfastInt3 &da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; return *this; };
	inline BfastInt3 &operator=(const int *da) { data[0] = da[0]; data[1] = da[1]; data[2] = da[2]; return *this; };

private:
	int data[3];
};
std::istream &operator>>(std::istream &strm, BfastInt3 &p);
std::ostream &operator<<(std::ostream &strm, const BfastInt3 &p);

/////////////////////////////////////////////////////////////////

class BfastInt3List {
 public:
  BfastInt3List();
  BfastInt3List(const BfastInt3List &that);
  ~BfastInt3List() { delete [] _int3s;};

  BfastInt3List &operator=(const BfastInt3List &that);

  inline BfastInt3 &index(int i) {return _int3s[i];};
  inline const BfastInt3 &index(int i) const {return _int3s[i];};
  inline BfastInt3 &operator[](int i) {return _int3s[i];};
  inline const BfastInt3 &operator[](int i) const {return _int3s[i];};

  inline unsigned int numInt3s() const {return _numInt3s;};
  void setNumInt3s(unsigned int num, int copyOld = 0, int exactSize = 1);
	void clear();

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numInt3s;
  int _numMalloc;
  BfastInt3 *_int3s;
};
std::istream &operator>>(std::istream &strm, BfastInt3List &p);
std::ostream &operator<<(std::ostream &strm, const BfastInt3List &p);
  
/////////////////////////////////////////////////////////////////


void randomize(BfastIndexList &list);

inline BfastReal absmin2 (BfastReal x, BfastReal y) {
  if (fabs(x) < fabs(y)) return x;
  return y;
}

inline BfastReal absmax2 (BfastReal x, BfastReal y) {
  if (fabs(x) > fabs(y)) return x;
  return y;
}

inline int sign(BfastReal x) {
  if (x>0.0) return 1;
	else return -1;
  //if (x<0.0) return -1;
  //if (x==0.0) return 0;
}

inline BfastReal max3 (BfastReal x, BfastReal y, BfastReal z) {
  if (x > y && x > z) return x;
  if (y > z) return y;
  return z;
}

inline BfastReal min3 (BfastReal x, BfastReal y, BfastReal z) {
  if (x < y && x < z) return x;
  if (y < z) return y;
  return z;
}

inline int max2 (int x, int y) {
  if (x>y) return x;
  else return y;
}

inline int min2 (int x, int y) {
  if (x<y) return x;
  else return y;
}

inline BfastReal min2 (BfastReal x, BfastReal y) {
  if (x<y) return x;
  else return y;
}

inline BfastReal max2 (BfastReal x, BfastReal y) {
  if (x>y) return x;
  else return y;
}

inline BfastReal max5(BfastReal x1, BfastReal x2, BfastReal x3, BfastReal x4, BfastReal x5) {
  BfastReal m = x1;
  if (x2 > m) m = x2;
  if (x3 > m) m = x3;
  if (x4 > m) m = x4;
  if (x5 > m) m = x5;
  return m;
}

BfastReal pow2(int p);

inline static
std::istream &eatChar(char c,std::istream &buf) {
  char r;
  buf >> r;
  if (r!=c) {
    buf.clear(buf.rdstate() | std::ios::failbit);
  }
  return buf;
}

inline static
std::istream &eatStr(const char *s,std::istream &buf) {
  while (*s != '\0') {
    eatChar(*s,buf);
    s++;
  }
  return buf;
}

// trace back in time (semi-Lagrangian)
inline BfastVector3 bfastTraceBack(const BfastVector3 &x, BfastVector3 (*evalVel)(const BfastVector3 &x, void *data),
																	 BfastReal dt, void *evalVelData=NULL) {
	BfastVector3 v = evalVel(x,evalVelData);
	BfastVector3 y = x-0.5*dt*v;
	v = evalVel(y,evalVelData);
	y = x-dt*v;
	return y;
}

// trace forward in time (fully Lagrangian)
inline BfastVector3 bfastTraceForward(const BfastVector3 &x, BfastVector3 (*evalVel)(const BfastVector3 &x, void *data),
																			BfastReal dt, void *evalVelData=NULL) {
	BfastVector3 v = evalVel(x,evalVelData);
	BfastVector3 y = x+0.5*dt*v;
	v = evalVel(y,evalVelData);
	y = x+dt*v;
	return y;
}

inline int bfastHash(int i, int j, int k) {
	return (i | j<<10 | k<<20);
}

inline int bfastHash(int *i) {
	return (i[0] | i[1]<<10 | i[2]<<20);
}

inline int bfastHash(BfastInt3 i) {
	return (i[0] | i[1]<<10 | i[2]<<20);
}

void computeNormals(const BfastVecList &meshPts, const BfastTriList &triangles, BfastVecList &vertexNormals, BfastVecList &faceNormals);
void bvtRead(char *fname, BfastVecList &meshPts, BfastTriList &triangles);
void gtDump(char *fname, const BfastVecList &meshPts, const BfastTriList &triangles);
void gtRead(char *fname, BfastVecList &meshPts, BfastTriList &triangles);
void bgtDump(char *fname, const BfastVecList &meshPts, const BfastTriList &triangles);

} // namespace SLC

#endif

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: bfastUtil.h,v $
// Revision 1.6  2006/06/16 21:07:14  adamb
// new version
//
// Revision 1.5  2006/01/15 01:43:52  adamb
// added rebuildTree
//
// Revision 1.4  2005/12/29 01:59:38  adamb
// Added void * to all passed in functions
//
// Revision 1.3  2005/11/08 19:45:05  adamb
// forced checkin
//
// Revision 1.2  2005/10/19 02:07:35  adamb
// seems to work
//
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.1  2005/08/10 01:33:25  adamb
// forced
//
// Revision 1.3  2005/07/20 01:46:58  adamb
// forced checkin of slc code before changes to hash function
//
// Revision 1.2  2005/06/10 20:08:47  adamb
// Various bug fixes, added iuc and ilc to cell output, rebuild vertMap after readining in a surface, fixed argc checking in the drivers.
//
// Revision 1.1.1.1  2005/05/25 05:40:59  adamb
// Initial Revision
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
