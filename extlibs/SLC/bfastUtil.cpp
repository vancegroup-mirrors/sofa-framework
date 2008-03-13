//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Useful Stuff
//   -- Code File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
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

#include "bfastUtil.h"
#include <iostream>
#include <fstream>

namespace SLC {

using namespace std;

BfastReal pow2(int p) {
  switch (p) {
	case -20: return 9.53674e-07;
	case -19: return 1.90735e-06;
	case -18: return 3.8147e-06;
	case -17: return 7.62939e-06;
	case -16: return 1.52588e-05;
	case -15: return 3.05176e-05;
	case -14: return 6.10352e-05;
	case -13: return 0.0001220703125;
	case -12: return 0.000244140625;
	case -11: return 0.00048828125;
	case -10: return 0.0009765625;
	case -9: return 0.001953125;
	case -8: return 0.00390625;
	case -7: return 0.0078125;
	case -6: return 0.015625;
	case -5: return 0.03125;
	case -4: return 0.0625;
	case -3: return 0.125;
	case -2: return 0.25;
	case -1: return 0.5;
	case 0: return 1;
	case 1: return 2;
	case 2: return 4;
	case 3: return 8;
	case 4: return 16;
	case 5: return 32;
	case 6: return 64;
	case 7: return 128;
	case 8: return 256;
	case 9: return 512;
	case 10: return 1024;
	case 11: return 2048;
	case 12: return 4096;
	case 13: return 8192;
	case 14: return 16384;
	case 15: return 32768;
	case 16: return 65536;
	case 17: return 131072;
	case 18: return 262144;
	case 19: return 524288;
	case 20: return 1048576;
  default:
    BfastReal ret = 1;
    if (p >= 0)
      for (int i=0; i<p; i++)
				ret *= 2.0;
    else
      for (int i=0; i< -p; i++)
				ret /= 2.0;
    return ret;
  }
}


//////////////////////////////////////////////////////////////////
// BfastIndexList
//////////////////////////////////////////////////////////////////

BfastIndexList::BfastIndexList() {
  _numValues = 0;
  _numMalloc = 0;
  _values = NULL;
}

BfastIndexList::BfastIndexList(const BfastIndexList &that) {
  //if (_values) delete [] _values; // BAD, VERY BAD!!!
  _numValues = that._numValues;
  _numMalloc = that._numMalloc;
  _values = new int[_numMalloc];
  memcpy (_values, that._values, _numValues*sizeof(int));
}

BfastIndexList &BfastIndexList::operator=(const BfastIndexList &that) {
  if (_values) delete [] _values;
  _numValues = that._numValues;
  _numMalloc = that._numMalloc;
  _values = new int[_numMalloc];
  memcpy(_values, that._values, _numValues*sizeof(int));
  return (*this);
}

void BfastIndexList::setNum(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&
	(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc)) &&
	(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }

  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numValues;
    int *oldList = _values;
    _numValues = num;
    _numMalloc = newMalloc;
    _values = new int[_numMalloc];
    if (copyOld) 
      memcpy(_values, oldList, ((num>oldSize)?(oldSize):(num))*sizeof(int));
    delete [] oldList;
  } else {
    _numValues = num;
  }
}

void BfastIndexList::add(int value) {
  setNum(_numValues+1,1,0);
  _values[_numValues-1] = value;
}

void BfastIndexList::add(BfastIndexList &list) {
  unsigned int j,i=_numValues;
  setNum(_numValues+list.getNum(),1,0);
	for (j=0; j<list.getNum(); j++, i++)
		_values[i] = list[j];
}

void BfastIndexList::add(int value, int index) {
  int i;
  setNum(_numValues+1,1,0);
  for (i=_numValues-1; i>index; i--)
    _values[i] = _values[i-1];
  _values[index] = value;
}

void BfastIndexList::swap(int index1, int index2) {
  int tmp = _values[index1];
  _values[index1] = _values[index2];
  _values[index2] = tmp;
}

void BfastIndexList::del(int index) {
  int i;
  for (i=index; i<_numValues-1; i++)
    _values[i] = _values[i+1];
  setNum(_numValues-1,1,0);
}

void BfastIndexList::delValue(int value) {
  int i;
  for (i=0; i<_numValues; i++)
    if (_values[i] == value)
      del(i);
}

void BfastIndexList::clear() {
  if (_numValues != 0) {
    _numValues = 0;
    _numMalloc = 0;
    if (_values) delete [] _values;
    _values = NULL;
  }
}

int BfastIndexList::contains(int value) {
  int i;
  int *v = _values;
  for (i=0; i<_numValues; i++, v++)
    if (*v == value) return 1;
  return 0;
}

int BfastIndexList::find(int value) {
  int i;
  int *v = _values;
  for (i=0; i<_numValues; i++, v++)
    if (*v == value) return i;
  return -1;
}

istream &operator>>(std::istream &strm, BfastIndexList &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNum(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const BfastIndexList &l) {
  strm << "[";
  strm << l.getNum();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.getNum();i++) {
    strm << l[i];
    if (i != (l.getNum()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void BfastIndexList::read(ifstream &in) {
	int l;
	in.read((char *)&l, sizeof(int));
	setNum(l);
	in.read((char*) _values, _numValues*sizeof(int));
}

void BfastIndexList::write(ofstream &out) {
	out.write((char*)&_numValues, sizeof(int));
	out.write((char*) _values, _numValues*sizeof(int));
}

void randomize(BfastIndexList &list) {
	int i=list.getNum(), j, tmp;

	for (i=0; i>0;) {
		j = rand()%(i--);
		tmp = list[i];
		list[i] = list[j];
		list[j] = tmp;
	}
		
}

//////////////////////////////////////////////////////////////////
// BfastRealList
//////////////////////////////////////////////////////////////////

BfastRealList::BfastRealList() {
  _numValues = 0;
  _numMalloc = 0;
  _values = NULL;
}

BfastRealList::BfastRealList(const BfastRealList &that) {
  if (_values) delete [] _values;
  _numValues = that._numValues;
  _numMalloc = that._numMalloc;
  _values = new BfastReal[_numMalloc];
  memcpy (_values, that._values, _numValues*sizeof(BfastReal));
}

BfastRealList &BfastRealList::operator=(const BfastRealList &that) {
  if (_values) delete [] _values;
  _numValues = that._numValues;
  _numMalloc = that._numMalloc;
  _values = new BfastReal[_numMalloc];
  memcpy(_values, that._values, _numValues*sizeof(BfastReal));
  return (*this);
}

void BfastRealList::setNumValues(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&
	(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc)) &&
	(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }
  
  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numValues;
    BfastReal *oldList = _values;
    _numValues = num;
    _numMalloc = newMalloc;
    _values = new BfastReal[_numMalloc];
		memset(_values, 0, _numMalloc*sizeof(BfastReal));
    if (copyOld) 
      memcpy(_values, oldList, ((num>oldSize)?(oldSize):(num))*sizeof(BfastReal));
    delete [] oldList;
  } else {
    _numValues = num;
  }
}

istream &operator>>(std::istream &strm, BfastRealList &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNumValues(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const BfastRealList &l) {
  strm << "[";
  strm << l.numValues();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.numValues();i++) {
    strm << l[i];
    if (i != (l.numValues()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void BfastRealList::read(ifstream &in) {
	int l;
	in.read((char *)&l, sizeof(int));
	setNumValues(l);
	in.read((char*) _values, _numValues*sizeof(BfastReal));
}

void BfastRealList::write(ofstream &out) {
	out.write((char *)&_numValues, sizeof(int));
	out.write((char*) _values, _numValues*sizeof(BfastReal));
}


//////////////////////////////////////////////////////////////////
// BfastTri
//////////////////////////////////////////////////////////////////
std::istream &operator>>(std::istream &strm, BfastTri &t) {
  //unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatStr("[",strm);
  strm >> t.a;
  eatChar(',',strm);
  strm >> t.b;
  eatChar(',',strm);
  strm >> t.c;
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}
std::ostream &operator<<(std::ostream &strm, const BfastTri &t) {
  strm << "[";
  strm << t.a;
  strm << ",";
  strm << t.b;
  strm << ",";
  strm << t.c;
  strm << "]";
  return strm;
}
void BfastTri::read(ifstream &in) {
	in.read((char*) &a, sizeof(int));
	in.read((char*) &b, sizeof(int));
	in.read((char*) &c, sizeof(int));
}
void BfastTri::write(ofstream &out) {
	out.write((char*) &a, sizeof(int));
	out.write((char*) &b, sizeof(int));
	out.write((char*) &c, sizeof(int));
}

//////////////////////////////////////////////////////////////////
// BfastTriList
//////////////////////////////////////////////////////////////////
BfastTriList::BfastTriList() {
  _numTriangles = 0;
  _numMalloc = 0;
  _triangles = NULL;
}

BfastTriList::BfastTriList(const BfastTriList &that) {
  if (_triangles) delete [] _triangles;
  _numTriangles = that._numTriangles;
  _numMalloc = that._numMalloc;
  _triangles = new BfastTri[_numMalloc];
  memcpy (_triangles, that._triangles, _numTriangles*sizeof(BfastTri));
}

BfastTriList &BfastTriList::operator=(const BfastTriList &that) {
  if (_triangles) delete [] _triangles;
  _numTriangles = that._numTriangles;
  _numMalloc = that._numMalloc;
  _triangles = new BfastTri[_numMalloc];
  memcpy(_triangles, that._triangles, _numTriangles*sizeof(BfastTri));
  return (*this);
}

int BfastTriList::add(int a, int b, int c) {
	const int i = numTriangles();
	setNumTriangles(numTriangles()+1,1,0);
	_triangles[i].a = a;
	_triangles[i].b = b;
	_triangles[i].c = c;
	return i;
}

void BfastTriList::setNumTriangles(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&
				(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc)) &&
				(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }
  
  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numTriangles;
    BfastTri *oldList = _triangles;
    _numTriangles = num;
    _numMalloc = newMalloc;
    _triangles = new BfastTri[_numMalloc];
    if (copyOld) 
      memcpy(_triangles, oldList, ((num>oldSize)?(oldSize):(num))*sizeof(BfastTri));
    delete [] oldList;
	} else {
    _numTriangles = num;
  }
}

#ifdef INVENTOR
SoIndexedFaceSet *BfastTriList::faceSet() {
	int i;
	SoIndexedFaceSet *faces = new SoIndexedFaceSet();
	faces->ref();
	faces->coordIndex.setNum(4*numTriangles());
	int32_t *ptr = faces->coordIndex.startEditing();
	for (i=0; i<numTriangles(); i++) {
		*(ptr++) = _triangles[i].a;
		*(ptr++) = _triangles[i].b;
		*(ptr++) = _triangles[i].c;
		*(ptr++) = -1;
	}
	faces->coordIndex.finishEditing();
	faces->unrefNoDelete();
	return faces;
}
#endif

istream &operator>>(std::istream &strm, BfastTriList &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNumTriangles(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const BfastTriList &l) {
  strm << "[";
  strm << l.numTriangles();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.numTriangles();i++) {
    strm << l[i];
    if (i != (l.numTriangles()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void BfastTriList::read(ifstream &in) {
	int i,l;
	in.read((char *)&l, sizeof(int));
	setNumTriangles(l);
	for (i=0; i<l; i++) _triangles[i].read(in);
}

void BfastTriList::write(ofstream &out) {
	int i;
	out.write((char *)&_numTriangles, sizeof(int));
	for (i=0; i<_numTriangles; i++) _triangles[i].write(out);
}


//////////////////////////////////////////////////////////////////
// BfastVecList
//////////////////////////////////////////////////////////////////
BfastVecList::BfastVecList() {
  _numVecs = 0;
  _numMalloc = 0;
  _vecs = NULL;
}

BfastVecList::BfastVecList(const BfastVecList &that) {
  if (_vecs) delete [] _vecs;
  _numVecs = that._numVecs;
  _numMalloc = that._numMalloc;
  _vecs = new BfastVector3[_numMalloc];
  memcpy (_vecs, that._vecs, _numVecs*sizeof(BfastVector3));
}

BfastVecList &BfastVecList::operator=(const BfastVecList &that) {
  if (_vecs) delete [] _vecs;
  _numVecs = that._numVecs;
  _numMalloc = that._numMalloc;
  _vecs = new BfastVector3[_numMalloc];
  memcpy(_vecs, that._vecs, _numVecs*sizeof(BfastVector3));
  return (*this);
}

void BfastVecList::clear() {
	if (_vecs) delete [] _vecs;
	_vecs = NULL;
	_numVecs = _numMalloc = 0;
}

void BfastVecList::setNumVecs(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&
				(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc)) &&
				(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }
  
  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numVecs;
    BfastVector3 *oldList = _vecs;
    _numVecs = num;
    _numMalloc = newMalloc;
    _vecs = new BfastVector3[_numMalloc];
    if (copyOld) 
      memcpy(_vecs, oldList, ((num>oldSize)?(oldSize):(num))*sizeof(BfastVector3));
    if (oldList) delete [] oldList;
  } else {
    _numVecs = num;
  }
}

#ifdef INVENTOR
SoCoordinate3 *BfastVecList::coord() {
	int i;
	SoCoordinate3 *coords = new SoCoordinate3();
	coords->ref();

	if (coords->point.getNum() != numVecs())
		coords->point.setNum(numVecs());

	SbVec3f *ptr = coords->point.startEditing();
	BfastVector3 *p = _vecs;
	for (i=0; i<numVecs(); i++,p++) 
		(ptr++)->setValue(float((*p)[0]), float((*p)[1]), float((*p)[2]));
	coords->point.finishEditing();
	coords->unrefNoDelete();
	return coords;
}
#endif

istream &operator>>(std::istream &strm, BfastVecList &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNumVecs(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const BfastVecList &l) {
  strm << "[";
  strm << l.numVecs();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.numVecs();i++) {
    strm << l[i];
    if (i != (l.numVecs()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void BfastVecList::read(ifstream &in) {
	int i,l;
	in.read((char*)&l, sizeof(int));
	setNumVecs(l);
	for (i=0; i<l; i++) {
		in.read((char*)&_vecs[i][0], sizeof(BfastReal));
		in.read((char*)&_vecs[i][1], sizeof(BfastReal));
		in.read((char*)&_vecs[i][2], sizeof(BfastReal));
	}
}

void BfastVecList::write(ofstream &out) {
	int i;
	out.write((char*)&_numVecs, sizeof(int));
	for (i=0; i<_numVecs; i++) {
		out.write((char*)&_vecs[i][0], sizeof(BfastReal));
		out.write((char*)&_vecs[i][1], sizeof(BfastReal));
		out.write((char*)&_vecs[i][2], sizeof(BfastReal));
	}
}

//////////////////////////////////////////////////////////////////
// BfastInt3List
//////////////////////////////////////////////////////////////////
istream &operator>>(std::istream &strm, BfastInt3 &v) {
	ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatChar('[',strm);
  strm >> v[0];
  eatChar(',',strm);
  strm >> v[1];
  eatChar(',',strm);
  strm >> v[2];
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}
  
ostream &operator<<(ostream &strm,const BfastInt3 &v) {
  strm << "[";
	strm << v[0]; strm << ",";
	strm << v[1]; strm << ",";
	strm << v[2]; strm << "]";
  return strm;
}

BfastInt3List::BfastInt3List() {
  _numInt3s = 0;
  _numMalloc = 0;
  _int3s = NULL;
}

BfastInt3List::BfastInt3List(const BfastInt3List &that) {
  if (_int3s) delete [] _int3s;
  _numInt3s = that._numInt3s;
  _numMalloc = that._numMalloc;
  _int3s = new BfastInt3[_numMalloc];
  memcpy (_int3s, that._int3s, _numInt3s*sizeof(BfastInt3));
}

BfastInt3List &BfastInt3List::operator=(const BfastInt3List &that) {
  if (_int3s) delete [] _int3s;
  _numInt3s = that._numInt3s;
  _numMalloc = that._numMalloc;
  _int3s = new BfastInt3[_numMalloc];
  memcpy(_int3s, that._int3s, _numInt3s*sizeof(BfastInt3));
  return (*this);
}

void BfastInt3List::clear() {
	if (_int3s) delete [] _int3s;
	_int3s = NULL;
	_numInt3s = _numMalloc = 0;
}

void BfastInt3List::setNumInt3s(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&
				(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc)) &&
				(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }
  
  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numInt3s;
    BfastInt3 *oldList = _int3s;
    _numInt3s = num;
    _numMalloc = newMalloc;
    _int3s = new BfastInt3[_numMalloc];
    if (copyOld) 
      memcpy(_int3s, oldList, ((num>oldSize)?(oldSize):(num))*sizeof(BfastInt3));
    if (oldList) delete [] oldList;
  } else {
    _numInt3s = num;
  }
}

istream &operator>>(std::istream &strm, BfastInt3List &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNumInt3s(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const BfastInt3List &l) {
  strm << "[";
  strm << l.numInt3s();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.numInt3s();i++) {
    strm << l[i];
    if (i != (l.numInt3s()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void BfastInt3List::read(ifstream &in) {
	int i,l;
	in.read((char*)&l, sizeof(int));
	setNumInt3s(l);
	for (i=0; i<l; i++) {
		in.read((char*)&_int3s[i][0], sizeof(BfastReal));
		in.read((char*)&_int3s[i][1], sizeof(BfastReal));
		in.read((char*)&_int3s[i][2], sizeof(BfastReal));
	}
}

void BfastInt3List::write(ofstream &out) {
	int i;
	out.write((char*)&_numInt3s, sizeof(int));
	for (i=0; i<_numInt3s; i++) {
		out.write((char*)&_int3s[i][0], sizeof(BfastReal));
		out.write((char*)&_int3s[i][1], sizeof(BfastReal));
		out.write((char*)&_int3s[i][2], sizeof(BfastReal));
	}
}

void computeNormals(const BfastVecList &meshPts, const BfastTriList &triangles, BfastVecList &vertexNormals, BfastVecList &faceNormals) {
	unsigned int i;
	faceNormals.setNumVecs(triangles.numTriangles());
	vertexNormals.setNumVecs(meshPts.numVecs()); //this zeros all the vectors
	for (i=0; i<vertexNormals.numVecs(); i++)
		vertexNormals[i] = 0.0;
	for (i=0; i<faceNormals.numVecs(); i++) {
		faceNormals[i] = cross(meshPts[triangles[i].b]-meshPts[triangles[i].a], 
													 meshPts[triangles[i].c]-meshPts[triangles[i].a]);
		vertexNormals[triangles[i].a]+=faceNormals[i];
		vertexNormals[triangles[i].b]+=faceNormals[i];
		vertexNormals[triangles[i].c]+=faceNormals[i];
	}

	for (i=0; i<vertexNormals.numVecs(); i++)
		normalize(vertexNormals[i]);
}

void bvtRead(char *fname, BfastVecList &meshPts, BfastTriList &triangles) {
	int version;
	ifstream fin (fname, ios::in | ios::binary);
	fin.read((char*)&version, sizeof(int));
	if (version == 1) {
		meshPts.read(fin);
		triangles.read(fin);
	} else {
		cerr << "Incompatible bvt file: "<<fname<<endl;
		exit(-1);
	}
	fin.close();
}

void gtDump(char *fname, const BfastVecList &meshPts, const BfastTriList &triangles) {
	unsigned int i;
	ofstream out;
	BfastVecList vertexNormals;
	BfastVecList faceNormals;
	computeNormals(meshPts,triangles,vertexNormals,faceNormals);
	out.open(fname);
	out<<"vertices: "<<meshPts.numVecs()<<std::endl;
	out<<"faces: "<<triangles.numTriangles()<<std::endl;

	for (i=0; i<meshPts.numVecs(); i++) 
		out<<"v "<<meshPts[i][0]<<" "<<meshPts[i][1]<<" "<<meshPts[i][2]<<" "<<vertexNormals[i][0]<<" "<<vertexNormals[i][1]<<" "<<vertexNormals[i][2]<<std::endl;
	
	for (i=0; i<triangles.numTriangles(); i++) 
		out<<"f "<<triangles[i].a+1<<" "<<triangles[i].b+1<<" "<<triangles[i].c+1<<std::endl;

	out.close();
}

void bgtDump(char *fname, const BfastVecList &meshPts, const BfastTriList &triangles) {
	unsigned int i;
	ofstream out;
	BfastVecList vertexNormals;
	BfastVecList faceNormals;
	computeNormals(meshPts,triangles,vertexNormals,faceNormals);
	out.open(fname, ios::out | ios::binary);
	int foo = meshPts.numVecs();
	out.write((char*)&foo, sizeof(int));
	foo = triangles.numTriangles();
	out.write((char*)&foo, sizeof(int));
	double bar;
	for (i=0; i<meshPts.numVecs(); i++) {
		bar = meshPts[i][0];
		out.write((char*)&bar, sizeof(double));
		bar = meshPts[i][1];
		out.write((char*)&bar, sizeof(double));
		bar = meshPts[i][2];
		out.write((char*)&bar, sizeof(double));
		bar = vertexNormals[i][0];
		out.write((char*)&bar, sizeof(double));
		bar = vertexNormals[i][1];
		out.write((char*)&bar, sizeof(double));
		bar = vertexNormals[i][2];
		out.write((char*)&bar, sizeof(double));
	}	
	for (i=0; i<triangles.numTriangles(); i++) {
		foo = triangles[i].a;
		out.write((char*)&foo, sizeof(int));
		foo = triangles[i].b;
		out.write((char*)&foo, sizeof(int));
		foo = triangles[i].c;
		out.write((char*)&foo, sizeof(int));
	}
	out.close();
}

void gtRead(char *fname, BfastVecList &meshPts, BfastTriList &triangles) {
	unsigned int i,j;
	ifstream in;
	BfastVecList vertexNormals;
	in.open(fname);
	eatStr("vertices:", in);
	in>>i;
	meshPts.setNumVecs(i);
	vertexNormals.setNumVecs(i);
	eatStr("faces:",in);
	in>>j;
	triangles.setNumTriangles(j);
	for (i=0; i<meshPts.numVecs(); i++) {
		eatChar('v', in);
		in>>meshPts[i][0]>>meshPts[i][1]>>meshPts[i][2]>>vertexNormals[i][0]>>vertexNormals[i][1]>>vertexNormals[i][2];
	}
	for (i=0; i<triangles.numTriangles(); i++) {
		eatChar('f', in);
		in>>triangles[i].a>>triangles[i].b>>triangles[i].c;
		triangles[i].a--;
		triangles[i].b--;
		triangles[i].c--;
	}
	in.close();
}

}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: bfastUtil.cpp,v $
// Revision 1.5  2006/06/16 21:07:13  adamb
// new version
//
// Revision 1.4  2006/01/15 01:43:51  adamb
// added rebuildTree
//
// Revision 1.3  2005/12/29 01:59:38  adamb
// Added void * to all passed in functions
//
// Revision 1.2  2005/11/08 19:45:05  adamb
// forced checkin
//
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.1  2005/08/10 01:33:25  adamb
// forced
//
// Revision 1.1.1.1  2005/05/25 05:40:59  adamb
// Initial Revision
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------

