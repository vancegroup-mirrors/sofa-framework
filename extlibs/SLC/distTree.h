//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Distance Tree
//   -- Header File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
//
// The distance tree is simply a balanced octree annotated with
// signed distance values.  This tree has two build functions,
// one takes a triangle mesh and splits the tree near the 
// triangles.  The other takes a function which returns a 
// signed distance at any query point.
//
// The individual octree cell's additionally contain a list
// of indices of triangles which intersect the cell.
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


#ifndef distTree_H
#define distTree_H

#include "bfast.h"
#include <fstream>
#include <iostream>
#include "bfastVector.h"
#include "bfastUtil.h"
#include <vector>

namespace SLC {

class DtTree;
class SplitCriterion;

////////////////////////////////////////////////////////////////

class DtCell {
public:
  int level;          // depth in the octree of this cell
  //BfastVector3 lc, uc;   // coordinates of the lower and upper corners of the cell
	//int ilc[3], iuc[3]; // integer coordinates of the lower and upper corner
  int vertices[8];    // indices into the octree's pts array of the vertices of this cell
  int parent;         // index of the parent of this cell
  int child[8];       // indices of the children of this cell
  //BfastReal dist;        // (approx) distance from the center of the cell to the surface
	BfastIndexList triangles; // list of triangles intersecting this cell
  DtCell() {};
  DtCell &operator=(const DtCell &that);
  DtCell(const DtCell &that);
  bool contains(DtTree *t, const BfastReal *x); // does x lie in this cell?
	int clamp(DtTree *t, BfastReal *x);           // make x lie in this cell.

  //const BfastVector3& lc(const DtTree* t) const { return t->pts[vertices[0]]; }
  //const BfastVector3& uc(const DtTree* t) const { return t->pts[vertices[7]]; }
  const BfastVector3& lc(const BfastVecList& pts) const { return pts[vertices[0]]; }
  const BfastVector3& uc(const BfastVecList& pts) const { return pts[vertices[7]]; }

	void read(std::ifstream &in);
	void write(std::ofstream &out);
};
std::istream &operator>>(std::istream &strm, DtCell &c);
std::ostream &operator<<(std::ostream &strm, const DtCell &c);

///////////////////////////////////////////////////////////////
class DtCellList {
 public:
  DtCellList();
  DtCellList(const DtCellList &that);
  ~DtCellList() { delete [] _cells;};

  DtCellList &operator=(const DtCellList &that);

  inline DtCell &index(int i) {return _cells[i];};
  inline const DtCell &index(int i) const {return _cells[i];};
  inline DtCell &operator[](int i) {return _cells[i];};
  inline const DtCell &operator[](int i) const {return _cells[i];};

  inline unsigned int numCells() const {return _numCells;};
  void setNumCells(unsigned int num, int copyOld = 0, int exactSize = 1);

	void read(std::ifstream &in);
	void write(std::ofstream &out);

 private:
  int _numCells;
  int _numMalloc;
  DtCell *_cells;
  BfastIndexList invalid;
};
std::istream &operator>>(std::istream &strm, DtCellList &p);
std::ostream &operator<<(std::ostream &strm, const DtCellList &p);
  
/////////////////////////////////////////////////////////////////

class DtTree {
public:
  BfastVector3 lc, uc; // upper and lower coordinates of the tree 
	               // (the domain is (lc,lc,lc)x(uc,uc,uc)
	int dim, dim2; // integer dimension and dimension squared
  int max_level; // how deep can the tree go?
  DtCellList cells; // list of cells
	BfastVecList pts; // octree vertices
	BfastInt3List ipts; // integer coordinates of octree vertices
  BfastRealList phi; // phi values at oct-tree points
	
  DtTree() {max_level=9; vertMap.clear();};
  ~DtTree() {vertMap.clear();};

	// function to build the tree.  
	void buildTree(BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
								 SplitCriterion &splitCriterion);
	void balanceTree();

  friend std::istream &operator>>(std::istream &strm, DtTree &t);
  friend std::ostream &operator<<(std::ostream &strm, const DtTree &t);

	// force x into the domain
	int clamp(BfastReal *x);
	int clamp(BfastVector3 &x);

	// find the cell containing x (maybe cell at level level).
	DtCell* findCell(const BfastVector3 &x);
	DtCell* findCell(const BfastVector3 &x, int level);
	int findCellIndex(const BfastVector3 &x);
	int findCellIndex(const BfastVector3 &x, int level);

	void read(std::ifstream &in);
	void write(std::ofstream &out);

  HASH_MAP<int,int> vertMap;

private:
	int nvertices;
	inline void addVertex(int p, int i, BfastReal x, BfastReal y, BfastReal z, int ix, int iy, int iz);
	inline void addCell(DtCell *cell, int parent, int level, int v0, int v1, int v2, int v3, int v4, int v5, int v6, int v7);
	inline void divideCell(int c, DtTree *tree);
};
std::istream &operator>>(std::istream &strm, DtTree &t);
std::ostream &operator<<(std::ostream &strm,const DtTree &t);

class SplitCriterion {
public:
	SplitCriterion() {};
	virtual ~SplitCriterion() {};
	virtual bool doSplit(DtTree *t, DtCell *c) = 0;
};

class TriangleSplitCriterion : public SplitCriterion {
public:
	TriangleSplitCriterion() {};
	~TriangleSplitCriterion() {};
	bool doSplit(DtTree *t, DtCell *c);
	BfastVecList *meshPts;
	BfastTriList *triangles;
	BfastVecList *faceNormals;
};

class PhiSplitCriterion : public SplitCriterion {
public:
	PhiSplitCriterion() {};
	~PhiSplitCriterion() {};
	PhiSplitCriterion(const PhiSplitCriterion &that) : SplitCriterion(that) {this->phiFunc = that.phiFunc; this->phiFuncData=that.phiFuncData;};
	bool doSplit(DtTree *t, DtCell *c);
	BfastReal (*phiFunc)(const BfastVector3 &x, void *data);
	void *phiFuncData;
};

class RegularSplitCriterion : public SplitCriterion {
public:
	RegularSplitCriterion() {};
	~RegularSplitCriterion() {};
	bool doSplit(DtTree * /*t*/ , DtCell * /*c*/) {return true;};
};

class TriangleCentroidSplitCriterion : public SplitCriterion {
public:
	TriangleCentroidSplitCriterion() {};
	~TriangleCentroidSplitCriterion() {};
	bool doSplit(DtTree *t, DtCell *c);
	BfastVecList *meshPts;
	BfastTriList *triangles;
#ifdef ROBUST
	BfastVecList *faceNormals;
	std::vector<BfastIndexList> cttriangles;
#endif
};

class ConcentricTripleSplitCriterion : public SplitCriterion {
public:
	ConcentricTripleSplitCriterion() {};
	~ConcentricTripleSplitCriterion() {};
	bool doSplit(DtTree *t, DtCell *c);
	DtTree *tree;
};



void buildTree (DtTree *tree, BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
								BfastVecList *meshPts, BfastTriList *triangles, BfastVecList *faceNormals);
void buildTree (DtTree *tree, BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
								BfastReal (*evalPhi)(const BfastVector3 &x, void *data), void *evalPhiData=NULL);

inline void DtTree::addVertex(int p, int i, BfastReal x, BfastReal y, BfastReal z, int ix, int iy, int iz) {
	vertMap[i] = p;
	pts.setNumVecs(p+1,1,0);
	ipts.setNumInt3s(p+1,1,0);
	pts[p].set(x,y,z);
	ipts[p].set(ix,iy,iz);
};

inline void DtTree::addCell(DtCell *cell, int parent, int level, int v0, int v1, int v2, int v3, int v4, int v5, int v6, int v7) {
	cell->parent = parent;
	cell->level = level;
	cell->vertices[0] = v0;
	cell->vertices[1] = v1;
	cell->vertices[2] = v2;
	cell->vertices[3] = v3;
	cell->vertices[4] = v4;
	cell->vertices[5] = v5;
	cell->vertices[6] = v6;
	cell->vertices[7] = v7;

	cell->child[0] = cell->child[1] = cell->child[2] = cell->child[3] =
		cell->child[4] = cell->child[5] = cell->child[6] = cell->child[7] = -1;
};

}

#endif

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: distTree.H,v $
// Revision 1.7  2006/06/16 21:07:14  adamb
// new version
//
// Revision 1.6  2006/01/15 01:43:52  adamb
// added rebuildTree
//
// Revision 1.5  2006/01/11 07:43:42  adamb
// changed width
//
// Revision 1.4  2005/12/29 01:59:38  adamb
// Added void * to all passed in functions
//
// Revision 1.3  2005/11/08 19:45:06  adamb
// forced checkin
//
// Revision 1.2  2005/10/19 02:07:35  adamb
// seems to work
//
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.1.1.1  2005/05/25 05:41:01  adamb
// Initial Revision
//
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
