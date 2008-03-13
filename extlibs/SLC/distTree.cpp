//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Distance Tree
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


#include "distTree.h"
#include "float.h"
#include <cstdlib>
#ifdef PCUBE
extern "C" {
 int
fast_polygon_intersects_cube(int nverts, const double verts[][3],
			const double polynormal[3],
			int already_know_verts_are_outside_cube,
			int already_know_edges_are_outside_cube);
}
#endif
#ifdef PCUBE_CPP
extern int
fast_polygon_intersects_cube(int nverts, const double verts[][3],
			const double polynormal[3],
			int already_know_verts_are_outside_cube,
			int already_know_edges_are_outside_cube);
#endif

namespace SLC {

#define EPS 1e-15
using namespace std;

const BfastReal width = 4;

//////////////////////////////////////////////////////////////////
// DtCell
//////////////////////////////////////////////////////////////////
bool DtCell::contains(DtTree *t, const BfastReal *x) {
  if (x[0] < t->pts[vertices[0]][0]-EPS || x[0] > t->pts[vertices[7]][0]+EPS) return false;
  if (x[1] < t->pts[vertices[0]][1]-EPS || x[1] > t->pts[vertices[7]][1]+EPS) return false;
  if (x[2] < t->pts[vertices[0]][2]-EPS || x[2] > t->pts[vertices[7]][2]+EPS) return false;
  return true;
}

int DtCell::clamp(DtTree *t, BfastReal *x) {
	int ret = 0;
  if (x[0] < t->pts[vertices[0]][0]) { x[0] = t->pts[vertices[0]][0]; ret += 1;}
  if (x[0] > t->pts[vertices[7]][0]) { x[0] = t->pts[vertices[7]][0]; ret += 2;}
  if (x[1] < t->pts[vertices[0]][1]) { x[1] = t->pts[vertices[0]][1]; ret += 4;}
  if (x[1] > t->pts[vertices[7]][1]) { x[1] = t->pts[vertices[7]][1]; ret += 8;}
  if (x[2] < t->pts[vertices[0]][2]) { x[2] = t->pts[vertices[0]][2]; ret += 16;}
  if (x[2] > t->pts[vertices[7]][2]) { x[2] = t->pts[vertices[7]][2]; ret += 32;}
  return ret;
}

std::istream &operator>>(std::istream &strm, DtCell &c) {
  //unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatStr("[",strm);
  strm >> c.level;
  eatChar(',',strm);
  strm >> c.vertices[0];
  eatChar(',',strm);
  strm >> c.vertices[1];
  eatChar(',',strm);
  strm >> c.vertices[2];
  eatChar(',',strm);
  strm >> c.vertices[3];
  eatChar(',',strm);
  strm >> c.vertices[4];
  eatChar(',',strm);
  strm >> c.vertices[5];
  eatChar(',',strm);
  strm >> c.vertices[6];
  eatChar(',',strm);
  strm >> c.vertices[7];
  eatChar(',',strm);
  strm >> c.parent;
  eatChar(',',strm);
  strm >> c.child[0];
  eatChar(',',strm);
  strm >> c.child[1];
  eatChar(',',strm);
  strm >> c.child[2];
  eatChar(',',strm);
  strm >> c.child[3];
  eatChar(',',strm);
  strm >> c.child[4];
  eatChar(',',strm);
  strm >> c.child[5];
  eatChar(',',strm);
  strm >> c.child[6];
  eatChar(',',strm);
  strm >> c.child[7];
  eatChar(',',strm);
  strm >> c.triangles;
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

std::ostream &operator<<(std::ostream &strm, const DtCell &c) {
  strm << "[";
  strm << c.level;
  strm << ",";
  strm << c.vertices[0];
  strm << ",";
  strm << c.vertices[1];
  strm << ",";
  strm << c.vertices[2];
  strm << ",";
  strm << c.vertices[3];
  strm << ",";
  strm << c.vertices[4];
  strm << ",";
  strm << c.vertices[5];
  strm << ",";
  strm << c.vertices[6];
  strm << ",";
  strm << c.vertices[7];
  strm << ",";
  strm << c.parent;
  strm << ",";
  strm << c.child[0];
  strm << ",";
  strm << c.child[1];
  strm << ",";
  strm << c.child[2];
  strm << ",";
  strm << c.child[3];
  strm << ",";
  strm << c.child[4];
  strm << ",";
  strm << c.child[5];
  strm << ",";
  strm << c.child[6];
  strm << ",";
  strm << c.child[7];
  strm << ",";
  strm << c.triangles;
  strm << "]";
  return strm;
}  

void DtCell::read(ifstream &in) {
	in.read((char*) &level, sizeof(int));
  in.read((char*) &vertices[0], sizeof(int));
  in.read((char*) &vertices[1], sizeof(int));
  in.read((char*) &vertices[2], sizeof(int));
  in.read((char*) &vertices[3], sizeof(int));
  in.read((char*) &vertices[4], sizeof(int));
  in.read((char*) &vertices[5], sizeof(int));
  in.read((char*) &vertices[6], sizeof(int));
  in.read((char*) &vertices[7], sizeof(int));
  in.read((char*) &parent, sizeof(int));
  in.read((char*) &child[0], sizeof(int));
  in.read((char*) &child[1], sizeof(int));
  in.read((char*) &child[2], sizeof(int));
  in.read((char*) &child[3], sizeof(int));
  in.read((char*) &child[4], sizeof(int));
  in.read((char*) &child[5], sizeof(int));
  in.read((char*) &child[6], sizeof(int));
  in.read((char*) &child[7], sizeof(int));
  triangles.read(in);
}

void DtCell::write(ofstream &out) {
	out.write((char*) &level, sizeof(int));
  out.write((char*) &vertices[0], sizeof(int));
  out.write((char*) &vertices[1], sizeof(int));
  out.write((char*) &vertices[2], sizeof(int));
  out.write((char*) &vertices[3], sizeof(int));
  out.write((char*) &vertices[4], sizeof(int));
  out.write((char*) &vertices[5], sizeof(int));
  out.write((char*) &vertices[6], sizeof(int));
  out.write((char*) &vertices[7], sizeof(int));
  out.write((char*) &parent, sizeof(int));
  out.write((char*) &child[0], sizeof(int));
  out.write((char*) &child[1], sizeof(int));
  out.write((char*) &child[2], sizeof(int));
  out.write((char*) &child[3], sizeof(int));
  out.write((char*) &child[4], sizeof(int));
  out.write((char*) &child[5], sizeof(int));
  out.write((char*) &child[6], sizeof(int));
  out.write((char*) &child[7], sizeof(int));
  triangles.write(out);
}

DtCell &DtCell::operator=(const DtCell &that) {
  level = that.level;
  vertices[0] = that.vertices[0];
  vertices[1] = that.vertices[1];
  vertices[2] = that.vertices[2];
  vertices[3] = that.vertices[3];
  vertices[4] = that.vertices[4];
  vertices[5] = that.vertices[5];
  vertices[6] = that.vertices[6];
  vertices[7] = that.vertices[7];
  parent = that.parent;
  child[0] = that.child[0];
  child[1] = that.child[1];
  child[2] = that.child[2];
  child[3] = that.child[3];
  child[4] = that.child[4];
  child[5] = that.child[5];
  child[6] = that.child[6];
  child[7] = that.child[7];
	triangles = that.triangles;
	return (*this);
}

DtCell::DtCell(const DtCell &that) {
  level = that.level;
  vertices[0] = that.vertices[0];
  vertices[1] = that.vertices[1];
  vertices[2] = that.vertices[2];
  vertices[3] = that.vertices[3];
  vertices[4] = that.vertices[4];
  vertices[5] = that.vertices[5];
  vertices[6] = that.vertices[6];
  vertices[7] = that.vertices[7];
  parent = that.parent;
  child[0] = that.child[0];
  child[1] = that.child[1];
  child[2] = that.child[2];
  child[3] = that.child[3];
  child[4] = that.child[4];
  child[5] = that.child[5];
  child[6] = that.child[6];
  child[7] = that.child[7];
}
  
  
//////////////////////////////////////////////////////////////////
// DtCellList
//////////////////////////////////////////////////////////////////
DtCellList::DtCellList() {
  _numCells = 0;
  _numMalloc = 0;
  _cells = NULL;
}

DtCellList::DtCellList(const DtCellList &that) {
  if (_cells) delete [] _cells;
  _numCells = that._numCells;
  _numMalloc = that._numMalloc;
  _cells = new DtCell[_numMalloc];
  memcpy (_cells, that._cells, _numCells*sizeof(DtCell));
}

DtCellList &DtCellList::operator=(const DtCellList &that) {
  if (_cells) delete [] _cells;
  _numCells = that._numCells;
  _numMalloc = that._numMalloc;
  _cells = new DtCell[_numMalloc];
  memcpy(_cells, that._cells, _numCells*sizeof(DtCell));
  return (*this);
}

void DtCellList::setNumCells(unsigned int num, int copyOld, int exactSize) {
  unsigned int oldMalloc = _numMalloc;
  unsigned int newMalloc = num;

  if (exactSize == 0) {
    if ((newMalloc < (oldMalloc))&&(newMalloc > (oldMalloc/2))) {
      newMalloc = oldMalloc;
    }
    if ((newMalloc > (oldMalloc))&&(newMalloc < (oldMalloc*2))) {
      newMalloc = oldMalloc*2;
    }
  }

  if (newMalloc != oldMalloc) {
    unsigned int oldSize = _numCells;
    DtCell *oldList = _cells;
    _numCells = num;
    _numMalloc = newMalloc;
    _cells = new DtCell[_numMalloc];
    if (copyOld) 
			for (unsigned int i=0; i<((num>oldSize)?(oldSize):(num)); i++)
				_cells[i] = oldList[i];
    delete [] oldList;
  } else {
    _numCells = num;
  }
}

istream &operator>>(std::istream &strm, DtCellList &l) {
  unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);

  eatStr("[",strm);
  strm >> newNum;
  eatStr(":",strm);
  
  if (strm.good()) {
    l.setNumCells(newNum,0);
    for(i=0;i<newNum;i++) {
      strm >> l[i];
      if (i != (newNum-1)) eatStr(",",strm);
    }
  }
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const DtCellList &l) {
  strm << "[";
  strm << l.numCells();
  strm << ":";
  unsigned int i;
  for(i=0;i<l.numCells();i++) {
    strm << l[i];
    if (i != (l.numCells()-1)) strm << ",";
  }
  strm << "]";
  return strm;
}

void DtCellList::read(ifstream &in) {
	int i,l;
	in.read((char *)&l, sizeof(int));
	setNumCells(l);
	for (i=0; i<l; i++) _cells[i].read(in);
}

void DtCellList::write(ofstream &out) {
	int i;
	out.write((char *)&_numCells, sizeof(int));
	for (i=0; i<_numCells; i++) _cells[i].write(out);
}

//////////////////////////////////////////////////////////////////
// DtTree
//////////////////////////////////////////////////////////////////

int DtTree::clamp(BfastVector3 &x) {
	return clamp(&(x[0]));
}

int DtTree::clamp(BfastReal *x) {
	int ret = 0;
  if (x[0] < lc[0]) { x[0] = lc[0]; ret += 1;}
  if (x[0] > uc[0]) { x[0] = uc[0]; ret += 2;}
  if (x[1] < lc[1]) { x[1] = lc[1]; ret += 4;}
  if (x[1] > uc[1]) { x[1] = uc[1]; ret += 8;}
  if (x[2] < lc[2]) { x[2] = lc[2]; ret += 16;}
  if (x[2] > uc[2]) { x[2] = uc[2]; ret += 32;}
  return ret;
}

DtCell* DtTree::findCell(const BfastVector3 &x) {
  int i;
  double mid[3];
	DtCell *c=&(cells[0]);

  while (c->child[0] != -1) {
    mid[0] = (pts[c->vertices[0]][0]+pts[c->vertices[7]][0])/2.0;
    mid[1] = (pts[c->vertices[0]][1]+pts[c->vertices[7]][1])/2.0;
    mid[2] = (pts[c->vertices[0]][2]+pts[c->vertices[7]][2])/2.0;
    i = 0;
    if (x[0] >= mid[0]) i+=4;
    if (x[1] >= mid[1]) i+=2;
    if (x[2] >= mid[2]) i+=1;
    c = &(cells[c->child[i]]);
  }
	return c;
}

int DtTree::findCellIndex(const BfastVector3 &x) {
  int i,index=0;
  double mid[3];
	DtCell *c=&(cells[0]);

  while (c->child[0] != -1) {
    mid[0] = (pts[c->vertices[0]][0]+pts[c->vertices[7]][0])/2.0;
    mid[1] = (pts[c->vertices[0]][1]+pts[c->vertices[7]][1])/2.0;
    mid[2] = (pts[c->vertices[0]][2]+pts[c->vertices[7]][2])/2.0;
    i = 0;
    if (x[0] >= mid[0]) i+=4;
    if (x[1] >= mid[1]) i+=2;
    if (x[2] >= mid[2]) i+=1;
    c = &(cells[index=(c->child[i])]);
  }
	return index;
}

DtCell* DtTree::findCell(const BfastVector3 &x, int level) {
  int i;
  double mid[3];
	DtCell *c=&(cells[0]);

  while (c->child[0] != -1 && c->level < level) {
    mid[0] = (pts[c->vertices[0]][0]+pts[c->vertices[7]][0])/2.0;
    mid[1] = (pts[c->vertices[0]][1]+pts[c->vertices[7]][1])/2.0;
    mid[2] = (pts[c->vertices[0]][2]+pts[c->vertices[7]][2])/2.0;
    i = 0;
    if (x[0] >= mid[0]) i+=4;
    if (x[1] >= mid[1]) i+=2;
    if (x[2] >= mid[2]) i+=1;
    c = &(cells[c->child[i]]);
  }
	return c;
}

int DtTree::findCellIndex(const BfastVector3 &x, int level) {
  int i,index=0;
  double mid[3];
	DtCell *c=&(cells[0]);

  while (c->child[0] != -1 && c->level < level) {
    mid[0] = (pts[c->vertices[0]][0]+pts[c->vertices[7]][0])/2.0;
    mid[1] = (pts[c->vertices[0]][1]+pts[c->vertices[7]][1])/2.0;
    mid[2] = (pts[c->vertices[0]][2]+pts[c->vertices[7]][2])/2.0;
    i = 0;
    if (x[0] >= mid[0]) i+=4;
    if (x[1] >= mid[1]) i+=2;
    if (x[2] >= mid[2]) i+=1;
    c = &(cells[index=(c->child[i])]);
  }
	return index;
}

istream &operator>>(std::istream &strm, DtTree &t) {
  //unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatStr("[",strm);
  strm >> t.max_level;
  eatChar(',',strm);
  strm >> t.lc;
  eatChar(',',strm);
  strm >> t.uc;
  eatChar(',',strm);
  strm >> t.cells;
  eatChar(',',strm);
  strm >> t.nvertices;
  eatChar(',',strm);
  strm >> t.pts;
  eatChar(',',strm);
  strm >> t.ipts;
  eatChar(',',strm);
  strm >> t.phi;
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const DtTree &t) {
  strm << "[";
  strm << t.max_level;
  strm << ",";
  strm << t.lc;
  strm << ",";
  strm << t.uc;
  strm << ",";
  strm << t.cells;
  strm << ",";
  strm << t.nvertices;
  strm << ",";
  strm << t.pts;
  strm << ",";
  strm << t.ipts;
  strm << ",";
  strm << t.phi;
  strm << "]";
  return strm;
}

void DtTree::read(std::ifstream &in) {
	int version; //, index;
	in.read((char*)&version, sizeof(int));
	if (in.fail()) {
		cerr<<"can't read from file"<<endl;
		exit(-1);
	}
	if (version == 1) {
		in.read((char*)&max_level, sizeof(int));
		in.read((char*)&(lc[0]), sizeof(BfastReal));
		in.read((char*)&(lc[1]), sizeof(BfastReal));
		in.read((char*)&(lc[2]), sizeof(BfastReal));
		in.read((char*)&(uc[0]), sizeof(BfastReal));
		in.read((char*)&(uc[1]), sizeof(BfastReal));
		in.read((char*)&(uc[2]), sizeof(BfastReal));
		cells.read(in);
		in.read((char*)&nvertices, sizeof(int));
		pts.read(in);
		ipts.read(in);
		phi.read(in);
	} else if (version == 2) {
		in.read((char*)&max_level, sizeof(int));
		in.read((char*)&(lc[0]), sizeof(BfastReal));
		in.read((char*)&(lc[1]), sizeof(BfastReal));
		in.read((char*)&(lc[2]), sizeof(BfastReal));
		in.read((char*)&(uc[0]), sizeof(BfastReal));
		in.read((char*)&(uc[1]), sizeof(BfastReal));
		in.read((char*)&(uc[2]), sizeof(BfastReal));
		cells.read(in);
		in.read((char*)&nvertices, sizeof(int));
		pts.read(in);
		ipts.read(in);
		phi.read(in);
	} else {		
		cerr<<"incompatible bdt file version="<<version<<endl;
	}
}

void DtTree::write(std::ofstream &out) {
	int version = 2;
	out.write((char*)&version, sizeof(int));
  out.write((char*)&max_level, sizeof(int));
  out.write((char*)&(lc[0]), sizeof(BfastReal));
  out.write((char*)&(lc[1]), sizeof(BfastReal));
  out.write((char*)&(lc[2]), sizeof(BfastReal));
  out.write((char*)&(uc[0]), sizeof(BfastReal));
  out.write((char*)&(uc[1]), sizeof(BfastReal));
  out.write((char*)&(uc[2]), sizeof(BfastReal));
  cells.write(out);
  out.write((char*)&nvertices, sizeof(int));
	pts.write(out);
	ipts.write(out);
  phi.write(out);
}


#ifndef PCUBE
bool TriangleSplitCriterion::doSplit(DtTree *, DtCell *) {

	cerr<<"Distance trees cannot be built from a triangle mesh without the pcube library"<<endl;
	exit(-1);
#else
bool TriangleSplitCriterion::doSplit(DtTree *t, DtCell *c) {

	unsigned int i,j;
	if (c->level == 0) {
		c->triangles.setNum(triangles->numTriangles());
		for (i=0; i<triangles->numTriangles(); i++) {
			c->triangles[i] = i;
		}
		return (c->triangles.getNum() != 0);
	}

	DtCell &parent = t->cells[c->parent];
	for (i=0; i<parent.triangles.getNum(); i++) {
		j = parent.triangles[i];
		BfastVector3 center=(t->pts[c->vertices[0]]+t->pts[c->vertices[7]])/2.0;
		double lh = (t->pts[c->vertices[7]][0]-t->pts[c->vertices[0]][0])*3.00;
		double tri[3][3];
		double n[3];
		tri[0][0]=((*meshPts)[(*triangles)[j].a][0]-center[0])/(lh);
		tri[0][1]=((*meshPts)[(*triangles)[j].a][1]-center[1])/(lh);
		tri[0][2]=((*meshPts)[(*triangles)[j].a][2]-center[2])/(lh);
		tri[1][0]=((*meshPts)[(*triangles)[j].b][0]-center[0])/(lh);
		tri[1][1]=((*meshPts)[(*triangles)[j].b][1]-center[1])/(lh);
		tri[1][2]=((*meshPts)[(*triangles)[j].b][2]-center[2])/(lh);
		tri[2][0]=((*meshPts)[(*triangles)[j].c][0]-center[0])/(lh);
		tri[2][1]=((*meshPts)[(*triangles)[j].c][1]-center[1])/(lh);
		tri[2][2]=((*meshPts)[(*triangles)[j].c][2]-center[2])/(lh);
		n[0] = (*faceNormals)[j][0];
		n[1] = (*faceNormals)[j][1];
		n[2] = (*faceNormals)[j][2];
		
		if (fast_polygon_intersects_cube(3,tri,n,0,0))
			c->triangles.add(j);
	}
	return (c->triangles.getNum() != 0);
#endif
}

bool TriangleCentroidSplitCriterion::doSplit(DtTree *t, DtCell *c) {
        unsigned int i,j;
        BfastIndexList lcttriangles;
        if (c->level == 0) {
                c->triangles.setNum(triangles->numTriangles());
                lcttriangles.setNum(triangles->numTriangles());
                for (i=0; i<triangles->numTriangles(); i++) {
                        c->triangles[i] = i;
#ifdef ROBUST
                        lcttriangles[i] = i;
#endif
                }
#ifdef ROBUST
                cttriangles.push_back(lcttriangles);
#endif
                return (c->triangles.getNum() != 0);
        }

        DtCell &parent = t->cells[c->parent];
        for (i=0; i<parent.triangles.getNum(); i++) {
                j = parent.triangles[i];
                BfastVector3 centroid=((*meshPts)[(*triangles)[j].a]+(*meshPts)[(*triangles)[j].b]+(*meshPts)[(*triangles)[j].c])/3.0;
                if (c->contains(t, &(centroid[0]))) c->triangles.add(j);
        }

#ifdef ROBUST
        for (i=0; i<cttriangles[c->parent].getNum(); i++) {
                j = cttriangles[c->parent][i];
                BfastVector3 center=(t->pts[c->vertices[0]]+t->pts[c->vertices[7]])/2.0;
                double lh = (t->pts[c->vertices[7]][0]-t->pts[c->vertices[0]][0])*3.00;
                double tri[3][3];
                double n[3];
                tri[0][0]=((*meshPts)[(*triangles)[j].a][0]-center[0])/(lh);
                tri[0][1]=((*meshPts)[(*triangles)[j].a][1]-center[1])/(lh);
                tri[0][2]=((*meshPts)[(*triangles)[j].a][2]-center[2])/(lh);
                tri[1][0]=((*meshPts)[(*triangles)[j].b][0]-center[0])/(lh);
                tri[1][1]=((*meshPts)[(*triangles)[j].b][1]-center[1])/(lh);
                tri[1][2]=((*meshPts)[(*triangles)[j].b][2]-center[2])/(lh);
                tri[2][0]=((*meshPts)[(*triangles)[j].c][0]-center[0])/(lh);
                tri[2][1]=((*meshPts)[(*triangles)[j].c][1]-center[1])/(lh);
                tri[2][2]=((*meshPts)[(*triangles)[j].c][2]-center[2])/(lh);
                n[0] = (*faceNormals)[j][0];
                n[1] = (*faceNormals)[j][1];
                n[2] = (*faceNormals)[j][2];
                if (fast_polygon_intersects_cube(3,tri,n,0,0))
                        lcttriangles.add(j);
        }
        cttriangles.push_back(lcttriangles);
        return (lcttriangles.getNum() != 0);
#else
				return (c->triangles.getNum() != 0);
#endif
}


bool PhiSplitCriterion::doSplit(DtTree *t, DtCell *c) {
	BfastVector3 x = 0.5*(t->pts[c->vertices[0]]+t->pts[c->vertices[7]]);
	double h = pow2(-c->level-1)*(t->uc[0]-t->lc[0]);
	return (width*h > fabs(phiFunc(x,phiFuncData)));
}

bool ConcentricTripleSplitCriterion::doSplit(DtTree *t, DtCell *c) {
	BfastVector3 z;
	BfastReal h;
	BfastVector3 y = 0.5*(t->pts[c->vertices[0]]+t->pts[c->vertices[7]]);
	DtCell *cell;

	h = (t->pts[c->vertices[7]][0]-t->pts[c->vertices[0]][0]);

	cell = tree->findCell(y, c->level);
	if (cell->triangles.getNum() != 0) {
		c->triangles = cell->triangles;
		return true;
	}
	
	z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;
	
	z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2];
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;
	
	z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2]-h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;

	z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2]+h;
	cell = tree->findCell(z, c->level);
	if (cell->triangles.getNum() != 0) return true;
	
	return false;
}


inline void DtTree::divideCell(int c, DtTree* /*tree*/) {
	int p,i,vert[19];

	// the following line fixes a problem with the apple compiler.
	cells.numCells();

	DtCell *cell = &(cells[c]);
	int l = cell->level+1;
	BfastVector3 lc = pts[cell->vertices[0]];
	BfastVector3 uc = pts[cell->vertices[7]];
	double h = (uc[0]-lc[0])/2.0;
	BfastInt3 ilc = ipts[cell->vertices[0]];
	BfastInt3 iuc = ipts[cell->vertices[7]];
	int ih = (iuc[0]-ilc[0])/2;

	int n = cells.numCells();
	cells.setNumCells(n+8,1,0);
	cell = &(cells[c]);
	cell->child[0] = n;
	cell->child[1] = n+1;
	cell->child[2] = n+2;
	cell->child[3] = n+3;
	cell->child[4] = n+4;
	cell->child[5] = n+5;
	cell->child[6] = n+6;
	cell->child[7] = n+7;
	p = pts.numVecs();
			
	i = bfastHash(ilc[0],ilc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[0] = p;
		addVertex(p,i,lc[0],lc[1],lc[2]+h,ilc[0],ilc[1],ilc[2]+ih);
		p++;
	} else vert[0] = vertMap[i];

	i = bfastHash(ilc[0],ilc[1]+ih,ilc[2]);
	if (!vertMap.count(i)) {
		vert[1] = p;
		addVertex(p,i,lc[0],lc[1]+h,lc[2],ilc[0],ilc[1]+ih,ilc[2]);
		p++;
	} else vert[1] = vertMap[i];
	
	i = bfastHash(ilc[0],ilc[1]+ih,ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[2] = p;
		addVertex(p,i,lc[0],lc[1]+h,lc[2]+h,ilc[0],ilc[1]+ih,ilc[2]+ih);
		p++;
	} else vert[2] = vertMap[i];

	i = bfastHash(ilc[0]+ih,ilc[1],ilc[2]);
	if (!vertMap.count(i)) {
		vert[3] = p;
		addVertex(p,i,lc[0]+h,lc[1],lc[2],ilc[0]+ih,ilc[1],ilc[2]);
		p++;
	} else vert[3] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,ilc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[4] = p;
		addVertex(p,i,lc[0]+h,lc[1],lc[2]+h,ilc[0]+ih,ilc[1],ilc[2]+ih);
		p++;
	} else vert[4] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,ilc[1]+ih,ilc[2]);
	if (!vertMap.count(i)) {
		vert[5] = p;
		addVertex(p,i,lc[0]+h,lc[1]+h,lc[2],ilc[0]+ih,ilc[1]+ih,ilc[2]);
		p++;
	} else vert[5] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,ilc[1]+ih,ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[6] = p;
		addVertex(p,i,lc[0]+h,lc[1]+h,lc[2]+h,ilc[0]+ih,ilc[1]+ih,ilc[2]+ih);
		p++;
	} else vert[6] = vertMap[i];
	
	i = bfastHash(ilc[0],ilc[1]+ih,iuc[2]);
	if (!vertMap.count(i)) {
		vert[7] = p;
		addVertex(p,i,lc[0],lc[1]+h,uc[2],ilc[0],ilc[1]+ih,iuc[2]);
		p++;
	} else vert[7] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,ilc[1],iuc[2]);
	if (!vertMap.count(i)) {
		vert[8] = p;
		addVertex(p,i,lc[0]+h,lc[1],uc[2],ilc[0]+ih,ilc[1],iuc[2]);
		p++;
	} else vert[8] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,ilc[1]+ih,iuc[2]);
	if (!vertMap.count(i)) {
		vert[9] = p;
		addVertex(p,i,lc[0]+h,lc[1]+h,uc[2],ilc[0]+ih,ilc[1]+ih,iuc[2]);
		p++;
	} else vert[9] = vertMap[i];
	
	i = bfastHash(ilc[0],iuc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[10] = p;
		addVertex(p,i,lc[0],uc[1],lc[2]+h,ilc[0],iuc[1],ilc[2]+ih);
		p++;
	} else vert[10] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,iuc[1],ilc[2]);
	if (!vertMap.count(i)) {
		vert[11] = p;
		addVertex(p,i,lc[0]+h,uc[1],lc[2],ilc[0]+ih,iuc[1],ilc[2]);
		p++;
	} else vert[11] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,iuc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[12] = p;
		addVertex(p,i,lc[0]+h,uc[1],lc[2]+h,ilc[0]+ih,iuc[1],ilc[2]+ih);
		p++;
	} else vert[12] = vertMap[i];
	
	i = bfastHash(ilc[0]+ih,iuc[1],iuc[2]);
	if (!vertMap.count(i)) {
		vert[13] = p;
		addVertex(p,i,lc[0]+h,uc[1],uc[2],ilc[0]+ih,iuc[1],iuc[2]);
		p++;
	} else vert[13] = vertMap[i];
	
	i = bfastHash(iuc[0],ilc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[14] = p;
		addVertex(p,i,uc[0],lc[1],lc[2]+h,iuc[0],ilc[1],ilc[2]+ih);
		p++;
	} else vert[14] = vertMap[i];
	
	i = bfastHash(iuc[0],ilc[1]+ih,ilc[2]);
	if (!vertMap.count(i)) {
		vert[15] = p;
		addVertex(p,i,uc[0],lc[1]+h,lc[2],iuc[0],ilc[1]+ih,ilc[2]);
		p++;
	} else vert[15] = vertMap[i];
	
	i = bfastHash(iuc[0],ilc[1]+ih,ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[16] = p;
		addVertex(p,i,uc[0],lc[1]+h,lc[2]+h,iuc[0],ilc[1]+ih,ilc[2]+ih);
		p++;
	} else vert[16] = vertMap[i];
	
	i = bfastHash(iuc[0],ilc[1]+ih,iuc[2]);
	if (!vertMap.count(i)) {
		vert[17] = p;
		addVertex(p,i,uc[0],lc[1]+h,uc[2],iuc[0],ilc[1]+ih,iuc[2]);
		p++;
	} else vert[17] = vertMap[i];
	
	i = bfastHash(iuc[0],iuc[1],ilc[2]+ih);
	if (!vertMap.count(i)) {
		vert[18] = p;
		addVertex(p,i,uc[0],uc[1],lc[2]+h,iuc[0],iuc[1],ilc[2]+ih);
		p++;
	} else vert[18] = vertMap[i];
			
	cell = &(cells[n]);
	addCell(cell,c,l,cells[c].vertices[0], vert[0], vert[1], vert[2], vert[3],
					vert[4],vert[5],vert[6]);
	cell++;
	addCell(cell,c,l, vert[0], cells[c].vertices[1], vert[2], vert[7], vert[4],
					vert[8],vert[6],vert[9]);
	cell++;
	addCell(cell,c,l,vert[1], vert[2], cells[c].vertices[2], vert[10], vert[5],
					vert[6],vert[11],vert[12]);
	cell++;
	addCell(cell,c,l,vert[2], vert[7], vert[10], cells[c].vertices[3], vert[6],
					vert[9],vert[12],vert[13]);
	cell++;
	addCell(cell,c,l,vert[3], vert[4], vert[5], vert[6], cells[c].vertices[4],
					vert[14],vert[15],vert[16]);
	cell++;
	addCell(cell,c,l,vert[4], vert[8], vert[6], vert[9], vert[14],
					cells[c].vertices[5],vert[16],vert[17]);
	cell++;
	addCell(cell,c,l,vert[5], vert[6], vert[11], vert[12], vert[15],
					vert[16],cells[c].vertices[6],vert[18]);
	cell++;
	addCell(cell,c,l,vert[6], vert[9], vert[12], vert[13], vert[16],
					vert[17],vert[18],cells[c].vertices[7]);
}

void DtTree::buildTree(BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
											 SplitCriterion &splitCriterion) {
  unsigned int c=0;
	BfastInt3 ilc, iuc;
  this->lc = input_lc;
  this->uc = input_uc;
	this->max_level = max_level;
	ilc = 0;
	iuc = (int)pow2(max_level);
	dim = (int)pow2(max_level)+1;
	dim2 = dim*dim;
	nvertices = dim2*dim;
	DtCell *cell;
	pts.setNumVecs(8);
	ipts.setNumInt3s(8);
  cells.setNumCells(1);
  cells[0].level = 0;
  cells[0].vertices[0] = 0;
  cells[0].vertices[1] = 1;
  cells[0].vertices[2] = 2;
  cells[0].vertices[3] = 3;
  cells[0].vertices[4] = 4;
  cells[0].vertices[5] = 5;
  cells[0].vertices[6] = 6;
  cells[0].vertices[7] = 7;
  cells[0].parent = -1;
  cells[0].child[0] = cells[0].child[1] = cells[0].child[2] = cells[0].child[3] =
    cells[0].child[4] = cells[0].child[5] = cells[0].child[6] = cells[0].child[7] = -1;

	pts[0].set(this->lc[0], this->lc[1], this->lc[2]);
	ipts[0].set(ilc[0], ilc[1], ilc[2]);
	vertMap[0] = 0;

	pts[1].set(this->lc[0], this->lc[1], this->uc[2]);
	ipts[1].set(ilc[0], ilc[1], iuc[2]);
	vertMap[(dim-1)<<20] = 1;
	
	pts[2].set(this->lc[0], this->uc[1], this->lc[2]);
	ipts[2].set(ilc[0], iuc[1], ilc[2]);
	vertMap[(dim-1)<<10] = 2;

	pts[3].set(this->lc[0], this->uc[1], this->uc[2]);
	ipts[3].set(ilc[0], iuc[1], iuc[2]);
	vertMap[(dim-1)<<10 | (dim-1)<<20] = 3;

	pts[4].set(this->uc[0], this->lc[1], this->lc[2]);
	ipts[4].set(iuc[0], ilc[1], ilc[2]);
	vertMap[dim-1] = 4;

	pts[5].set(this->uc[0], this->lc[1], this->uc[2]);
	ipts[5].set(iuc[0], ilc[1], iuc[2]);
	vertMap[(dim-1) | (dim-1)<<20] = 5;

	pts[6].set(this->uc[0], this->uc[1], this->lc[2]);
	ipts[6].set(iuc[0], iuc[1], ilc[2]);
	vertMap[(dim-1) | (dim-1)<<10] = 6;

	pts[7].set(this->uc[0], this->uc[1], this->uc[2]);
	ipts[7].set(iuc[0], iuc[1], iuc[2]);
	vertMap[(dim-1) | (dim-1)<<10 | (dim-1)<<20] = 7;

	while (c < cells.numCells()) {
		cell = &(cells[c]);
		if (splitCriterion.doSplit(this, cell) && cell->level < max_level)
			divideCell(c,this);
		c++;
	} 
	cells.setNumCells(cells.numCells(),1,1);
	pts.setNumVecs(pts.numVecs(),1,1);
	ipts.setNumInt3s(ipts.numInt3s(),1,1);
	for (c=0; c<cells.numCells(); c++)
		cells[c].triangles.setNum(cells[c].triangles.getNum(),1,1);
}

void DtTree::balanceTree() {
	unsigned int c=0;
	DtCell *cell1, *cell2;
	double LEPS = 1e-100;
	int done = false;
	BfastVector3 x, v[8];

	while (!done) {
		done = true;
		while (c < cells.numCells()) {
			cell1 = &(cells[c]);
			v[0] = pts[cell1->vertices[0]];
			v[1] = pts[cell1->vertices[1]];
			v[2] = pts[cell1->vertices[2]];
			v[3] = pts[cell1->vertices[3]];
			v[4] = pts[cell1->vertices[4]];
			v[5] = pts[cell1->vertices[5]];
			v[6] = pts[cell1->vertices[6]];
			v[7] = pts[cell1->vertices[7]];
			
			x.set(v[0][0]-LEPS, v[0][1]+LEPS, v[0][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[1][0]-LEPS, v[1][1]+LEPS, v[1][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[2][0]-LEPS, v[2][1]-LEPS, v[2][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[3][0]-LEPS, v[3][1]-LEPS, v[3][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[4][0]+LEPS, v[4][1]+LEPS, v[4][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[5][0]+LEPS, v[5][1]+LEPS, v[5][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[6][0]+LEPS, v[6][1]-LEPS, v[6][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[7][0]+LEPS, v[7][1]-LEPS, v[7][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[0][0]+LEPS, v[0][1]-LEPS, v[0][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[1][0]+LEPS, v[1][1]-LEPS, v[1][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[4][0]-LEPS, v[4][1]-LEPS, v[4][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[5][0]-LEPS, v[5][1]-LEPS, v[5][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[2][0]+LEPS, v[2][1]+LEPS, v[2][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[3][0]+LEPS, v[3][1]+LEPS, v[3][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[6][0]-LEPS, v[6][1]+LEPS, v[6][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[6][0]-LEPS, v[6][1]+LEPS, v[6][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[0][0]+LEPS, v[0][1]+LEPS, v[0][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[2][0]+LEPS, v[2][1]-LEPS, v[2][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[4][0]-LEPS, v[4][1]+LEPS, v[4][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[6][0]-LEPS, v[6][1]+LEPS, v[6][2]-LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[1][0]+LEPS, v[1][1]+LEPS, v[1][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[3][0]+LEPS, v[3][1]-LEPS, v[3][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[5][0]-LEPS, v[5][1]+LEPS, v[5][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			x.set(v[7][0]-LEPS, v[7][1]-LEPS, v[7][2]+LEPS);
			cell2 = findCell(x);
			if (cell2->level > cell1->level+1) { divideCell(c,this); done = false; continue; }

			c++;
		}
	}
}

void buildTree (DtTree *tree, BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
								BfastVecList *meshPts, BfastTriList *triangles, BfastVecList *faceNormals) {
	TriangleSplitCriterion splitCriterion;
	splitCriterion.meshPts = meshPts;
	splitCriterion.triangles = triangles;
	splitCriterion.faceNormals = faceNormals;
	tree->buildTree(input_lc, input_uc, max_level, splitCriterion);
}

void buildTree (DtTree *tree, BfastVector3 input_lc, BfastVector3 input_uc, int max_level,
								BfastReal (*evalPhi)(const BfastVector3 &x, void *data), void *evalPhiData) {
	unsigned int i;
	BfastVector3 *x;
	BfastReal *p;
	PhiSplitCriterion splitCriterion;
	splitCriterion.phiFunc = evalPhi;
	splitCriterion.phiFuncData = evalPhiData;
	tree->buildTree(input_lc, input_uc, max_level, splitCriterion);

	tree->phi.setNumValues(tree->pts.numVecs());
	x = &(tree->pts[0]);
	p = &(tree->phi[0]);
	for (i=0; i<tree->pts.numVecs(); i++, p++, x++) 
		(*p) = evalPhi(*x, evalPhiData);
}

}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: distTree.cpp,v $
// Revision 1.7  2006/06/16 21:07:13  adamb
// new version
//
// Revision 1.6  2006/01/15 01:43:51  adamb
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
// Revision 1.4  2005/07/20 05:09:01  adamb
// first compiling fluid simulator
//
// Revision 1.3  2005/07/20 02:35:11  adamb
// forced checkin of slc code *after* changes to hash function
//
// Revision 1.2  2005/06/10 20:08:47  adamb
// Various bug fixes, added iuc and ilc to cell output, rebuild vertMap after readining in a surface, fixed argc checking in the drivers.
//
// Revision 1.1.1.1  2005/05/25 05:41:01  adamb
// Initial Revision
//
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
