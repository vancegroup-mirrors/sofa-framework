//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// SLC Surface Tracker
//   -- Header File
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

#ifndef slcSurface_H
#define slcSurface_H

#include "bfast.h"
#include <fstream>
#include <iostream>
#include "bfastVector.h"
#include "bfastUtil.h"
#include "distTree.h"

namespace SLC {

class DtFMMHeap;

#ifdef WIN32
#if _MSC_VER >= 1400
class __slc_hash_unsigned_long_long : public stdext::hash_compare<unsigned long long> {
#else
class __slc_hash_unsigned_long_long : public std::hash_compare<unsigned long long> {
#endif
public:
	inline std::size_t operator()(const unsigned long long& __s) const {
		unsigned long __h = (long)((__s >> 32) + (__s & 0xFFFFFFFF));
		return std::size_t(__h);
	};
	inline bool operator()(const unsigned long long& __s1, const unsigned long long& __s2) const {
		return __s1 < __s2;
	};
	enum { // parameters for hash table
		bucket_size = 4, // 0 < bucket_size
		min_buckets = 8
	}; // min_buckets = 2 ^^ N, 0 < N
};
#else
class __slc_hash_unsigned_long_long {
public:
	inline std::size_t operator()(const unsigned long long &__s) const {
		unsigned long __h = (__s >> 32) + (__s & 0xFFFFFFFF);
		return std::size_t(__h);
	};
};
#endif

class SlcSurface {
public:
	SlcSurface(DtTree *tree);
	~SlcSurface();

	DtTree *tree;
	BfastTriList triangles; // triangles in polygon mesh
	BfastVecList meshPts; // points in polygon mesh
	BfastVecList normals; // normals (at points) in polygon mesh
	BfastVecList psuedoNormals; // angle weighted normals (at points) in polygon mesh
	BfastVecList faceNormals; // face normals in polygon mesh

	bool builtFromTriangles; // was the tree constructed from triangles or a phi func.

	HASH_SET<int> visitedTris;
  HASH_MAP<unsigned long long,unsigned int, __slc_hash_unsigned_long_long> edgeList;

	// Marching cubes functions.
	// vertid creates new vertices or finds already created ones.
	// dotet creates triangles for a tet through which the surface passes.
	// contourTree drives the contouring.

	int vertid(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), int a, int b, BfastVector3 x, BfastVector3 y, 
						 bool apos, bool bpos, BfastReal aphi, BfastReal bphi, void *evalPhiData=NULL);
	void dotet(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), DtCell *cell,  
						 int a, int b, int c, int d, BfastVector3 w, BfastVector3 x, BfastVector3 y, BfastVector3 z, void *evalPhiData=NULL);
	void contourTree(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), void *evalPhiData=NULL);

	// end marching cubes functions

	void redistance(); // redistance the tree.

	void computeNormals();
	void computePsuedoNormals();

	// a variety of signed-distance evaluation routines.  x is the
	// evaluation point.  If interpolate is true, the evaluation is
	// gotten from linear interpolation of the values stored at
	// octree vertices.  s & t are barycentric coordinates and face is the 
	// triangle of the nearest point in the mesh.  creval fits a cubic
	// catmull-rom spline and evaluates that.

	BfastReal eval(const BfastReal *x, bool interpolate = false);
	BfastReal eval(const BfastReal *x, BfastReal &s, BfastReal &t, int &face, bool interpolate = false);
	BfastReal eval(const BfastVector3 &x, bool interpolate = false);
	BfastReal eval(const BfastVector3 &x, BfastReal &s, BfastReal &t, int &face, bool interpolate = false);
	BfastReal creval(const BfastVector3 &x);
	BfastReal creval(const BfastReal *x);


	// evaluate the gradient at x.
	BfastVector3 grad(const BfastVector3 &x);
	BfastVector3 grad(const BfastReal *x);

	// distance from a point to a triangle.  This searches the entire triangle
	// list of cell.  The second version is more
	// complicated because it is meant to be called for each cell in the concentric
	// triple and has to deal with sharp features, etc.
	BfastReal pointToTri(DtCell *c, const BfastVector3 &x, BfastReal &phi);
	BfastReal pointToTri(DtCell *cell, const BfastVector3 &x, BfastReal &phi, int &sgn,
											BfastReal &s, BfastReal &t, int &face, BfastReal &dp, int edge[2]);

	// i/o functions
	void read(std::ifstream &in);
	void write(std::ofstream &out);
	void objDump(const char *fname);
	void ribDump(const char *fname);
	void bvtDump(const char *fname);
	void bvtRead(const char *fname);
	void bdtDump(const char *fname);
	void bdtRead(const char *fname);

	void setMaxFunc(BfastReal (*maxFunc)(const BfastReal *x, void *data), void *data=NULL);
	void setMinFunc(BfastReal (*minFunc)(const BfastReal *x, void *data), void *data=NULL);
	
	void rebuildTree();

#ifdef INVENTOR
	SoSeparator *ivTree();
	void ivDump(const char *fname);
#endif

private:	
	void addToFMMHeap(int i, int j, int k, DtFMMHeap &heap,
										short *levels, bool *accepted, BfastRealList &newPhi, int foo);
	void addToFMMHeap(BfastInt3 ipt, DtFMMHeap &heap,
										short *levels, bool *accepted, BfastRealList &newPhi, int foo);
	BfastReal exacteval(DtCell *c, const BfastVector3 &x, BfastReal &s, BfastReal &t, 
									 int &face);
	BfastReal interpolate(DtCell *c, const BfastVector3 &x);

	// whenever phi is evaluated, the returned result will be min(minFunc, phi)
	BfastReal (*minFunc)(const BfastReal *x, void *data);
	void *minFuncData;

	// whenever phi is evaluated, the returned result will be max(maxFunc, phi)
	BfastReal (*maxFunc)(const BfastReal *x, void *data);
	void *maxFuncData;

};

void readObjFile(const char *fname, BfastVecList &pts, BfastTriList &triangles);

#ifdef INVENTOR
#include <Inventor/SoOutput.h>
#include <Inventor/actions/SoWriteAction.h>
int bfastWriteIVFile(const char *filename, SoNode *node);
#endif

// Fast marching method code.
class DtFMMHeapEntry {
 public:
  DtFMMHeapEntry(){};
  DtFMMHeapEntry(int x, int y, int z, BfastReal phi) {i = x, j = y, k = z, this->phi = phi;};
  DtFMMHeapEntry &operator=(const DtFMMHeapEntry &e){i=e.i,j=e.j,k=e.k,phi=e.phi; return *this;};
  int i,j,k;
  BfastReal phi;
};
  
class DtFMMHeap {
 public:
  DtFMMHeap(int dim, int dim2);
  ~DtFMMHeap();
  void addToHeap(int i, int j, int k, BfastReal phi);
  DtFMMHeapEntry removeFromHeap();
  inline unsigned int numEntries() const {return _numEntries;};
  void setNumEntries(unsigned int num, int copyOld = 0, int exactSize = 1);
  DtFMMHeapEntry *findEntry(int i, int j, int k);
 private:
  int dim, dim2;
  void swap(int i, int j);
  int _numEntries;
  DtFMMHeapEntry *_entries;
  int _numMalloc;

  HASH_MAP<int,int> mymap;
};

////////////////////////////////////////////////////////////////

}

#endif


//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: slcSurface.H,v $
// Revision 1.4  2006/01/15 01:43:52  adamb
// added rebuildTree
//
// Revision 1.3  2005/12/29 01:59:38  adamb
// Added void * to all passed in functions
//
// Revision 1.2  2005/11/08 19:45:06  adamb
// forced checkin
//
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.3  2005/07/20 01:46:58  adamb
// forced checkin of slc code before changes to hash function
//
// Revision 1.2  2005/06/10 20:08:47  adamb
// Various bug fixes, added iuc and ilc to cell output, rebuild vertMap after readining in a surface, fixed argc checking in the drivers.
//
// Revision 1.1.1.1  2005/05/25 05:41:04  adamb
// Initial Revision
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
