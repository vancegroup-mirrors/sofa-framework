//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// SLC Surface Tracker
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
#include "slcSurface.h"
extern "C" {
#include "pcube/pcube.h"
}


namespace SLC {

#define EPS 1e-15
using namespace std;

const BfastReal width = 3;

double orient3d(BfastReal *pa, BfastReal *pb, BfastReal *pc, BfastReal *pd) {
  double adx, bdx, cdx;
  double ady, bdy, cdy;
  double adz, bdz, cdz;

  adx = pa[0] - pd[0];
  bdx = pb[0] - pd[0];
  cdx = pc[0] - pd[0];
  ady = pa[1] - pd[1];
  bdy = pb[1] - pd[1];
  cdy = pc[1] - pd[1];
  adz = pa[2] - pd[2];
  bdz = pb[2] - pd[2];
  cdz = pc[2] - pd[2];

  return adx * (bdy * cdz - bdz * cdy)
       + bdx * (cdy * adz - cdz * ady)
       + cdx * (ady * bdz - adz * bdy);
}


SlcSurface::SlcSurface(DtTree *tree) {
	this->tree = tree;
	minFunc = NULL;
	maxFunc = NULL;
	minFuncData = NULL;
	maxFuncData = NULL;
	edgeList.clear();
	visitedTris.clear();
};

SlcSurface::~SlcSurface() {
	edgeList.clear();
	visitedTris.clear();
}


BfastReal SlcSurface::exacteval(DtCell *c, const BfastVector3 &x, BfastReal &s, BfastReal &t, 
														 int &face) {
	//BfastReal d=BFAST_REAL_MAX;
	BfastVector3 y,z;
	DtCell *oc = c;
	BfastReal h;
	BfastReal up,down,left,right,front,back;
	BfastReal phi = BFAST_REAL_MAX;
	BfastReal dp = 0.0;
	int edge[2] = {-1,-1};
	int sgn = 0;
	if (builtFromTriangles) visitedTris.clear();

	right = tree->pts[c->vertices[7]][0]-x[0];
	left = x[0]-tree->pts[c->vertices[0]][0];
	up = tree->pts[c->vertices[7]][1]-x[1];
	down = x[1]-tree->pts[c->vertices[0]][1];
	back = tree->pts[c->vertices[7]][2]-x[2];
	front = x[2]-tree->pts[c->vertices[0]][2];

	y = (tree->pts[c->vertices[7]]+tree->pts[c->vertices[0]])/2.0;
		
	h = (tree->pts[c->vertices[7]][0]-tree->pts[c->vertices[0]][0]);
	if (c->triangles.getNum() != 0) 
		pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	
	if (phi > left) {
		z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > down) {
		z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > front) {
		z[0]=y[0]; z[1]=y[1]; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > back) {
		z[0]=y[0]; z[1]=y[1]; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > up) {
		z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > right) {
		z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > down) {
		z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > front) {
		z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > back) {
		z[0]=y[0]-h; z[1]=y[1]; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > up) {
		z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > down && phi > front) {
		z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > down && phi > back) {
		z[0]=y[0]; z[1]=y[1]-h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > up && phi > front) {
		z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > up && phi > back) {
		z[0]=y[0]; z[1]=y[1]+h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > right && phi > down) {
		z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}


	if (phi > right && phi > front) {
		z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > right && phi > back) {
		z[0]=y[0]+h; z[1]=y[1]; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > right && phi > up) {
		z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2];
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > down && phi > front) {
		z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > left && phi > down && phi > back) {
		z[0]=y[0]-h; z[1]=y[1]-h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}


	if (phi > left && phi > up && phi > front) {
		z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}


	if (phi > left && phi > up && phi > back) {
		z[0]=y[0]-h; z[1]=y[1]+h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	
	if (phi > right && phi > down && phi > front) {
		z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}


	if (phi > right && phi > down && phi > back) {
		z[0]=y[0]+h; z[1]=y[1]-h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}
	
	if (phi > right && phi > up && phi > front) {
		z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2]-h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}

	if (phi > right && phi > up && phi > back) {
		z[0]=y[0]+h; z[1]=y[1]+h; z[2]=y[2]+h;
		c = tree->findCell(z);
		if (c->triangles.getNum() != 0 && c->level == tree->max_level) 
			pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	}
	
	if (phi!=BFAST_REAL_MAX) {
		if (oc->triangles.getNum() == 0) 
			sgn = sign(tree->phi[oc->vertices[0]]);
		else if (phi > h) 
			sgn = sign(interpolate(oc,x));
		return sgn*phi;
	} else 
		return interpolate(oc,x);

}

BfastReal SlcSurface::pointToTri(DtCell *c, const BfastVector3 &x, BfastReal &phi) {
	phi = BFAST_REAL_MAX;
	BfastReal s = 0.0, t = 0.0;
	int face = -1;
	BfastReal dp = 0.0;
	int sgn = 0;
	int edge[2]={-1,-1};
	pointToTri(c,x,phi,sgn,s,t,face,dp,edge);
	if (sgn != 0) phi *= sgn;
	return phi;
}

BfastReal SlcSurface::pointToTri(DtCell *cell, const BfastVector3 &x, BfastReal &phi, int &sgn,
															BfastReal &s, BfastReal &t, int &face, BfastReal &dp, int edge[2]) {
	unsigned int i,j;
	BfastVector3 P1, P2, P3, D, E0, E1, X(x), C;
	BfastReal ls,lt,a,b,c,d,e,f,det,numer,denom,est;
	int onEdge=0; 
	int point=-1;
	
	for (i=0; i<cell->triangles.getNum(); i++) {
		j = cell->triangles[i];
		if (builtFromTriangles) 
			if (visitedTris.count(j)) continue;
			else visitedTris.insert(j);
		onEdge = 0;
		point = -1;
		P1 = meshPts[triangles[j].a];
		P2 = meshPts[triangles[j].b];
		P3 = meshPts[triangles[j].c];
		E0 = P2-P1;
		E1 = P3-P1;
		D=P1-X; 
		a = dot(E0,E0);
		b = dot(E0,E1);
		c = dot(E1,E1); 
		d = dot(E0,D);
		e = dot(E1,D); 
		f = dot(D,D);
		det = a*c-b*b;
		ls = b*e-c*d;
		lt = b*d-a*e;

		if (ls+lt <= det) {
			if (ls<0) {
				if (lt<0) {
					if (d<0.0) {
						lt = 0.0;
						if (-d>=a) {ls = 1.0; point=triangles[j].b;}
						else {ls = -d/a; onEdge=1;}
					} else {
						ls = 0.0;
						if (e >= 0.0) {lt = 0.0; point=triangles[j].a;}
						else if (-e >= c) {lt = 1.0; point=triangles[j].c;}
						else {lt = -e/c; onEdge=2;}
					}
				} else {
					ls = 0.0;
					if (e >= 0.0) {lt = 0.0; point=triangles[j].a;}
					else if (-e >= c) {lt = 1.0; point=triangles[j].c;}
					else {lt = -e/c; onEdge=2;}
				}
			} else if (lt <0) {
				lt = 0;
				if (d >= 0) {ls = 0; point=triangles[j].a;}
				else if (-d >= a) {ls = 1; point=triangles[j].b;}
				else {ls = -d/a; onEdge=1;}
			} else {
				if (det != 0) {ls /= det; lt /= det;}
				else {point=triangles[j].a;}
			}
		} else {
			BfastReal tmp0;
			BfastReal tmp1;
			if (ls < 0) {
				tmp0 = b+d;
				tmp1 = c+e;
				if (tmp1 > tmp0) {
					numer = tmp1-tmp0;
					denom = a-2.0*b+c;
					if (numer >= denom) {ls = 1.0; lt = 0.0; point=triangles[j].b;} 
					else {ls = numer/denom; lt = 1.0-ls; onEdge=3;}
				} else {
					ls = 0;
					if (tmp1 <= 0.0) {lt = 1.0; point=triangles[j].c;}
					else if (e >= 0) {lt = 0.0; point=triangles[j].a;}
					else {lt = -e/c; onEdge=2;}
				}
			} else if (lt < 0) {
				tmp0 = b+e;
				tmp1 = a+d;
				if (tmp1 > tmp0) {
					numer = tmp1-tmp0;
					denom = a-2.0*b+c;
					if (numer >= denom) {lt = 1.0; ls = 0.0; point=triangles[j].c;}
					else {lt = numer/denom; ls = 1.0-lt; onEdge=3;}
				} else {
					lt = 0.0;
					if (tmp1 <= 0) {ls = 1.0; point=triangles[j].b;}
					else if (d >= 0.0) {ls = 0.0; point=triangles[j].a;}
					else {ls = -d/a; onEdge=1;}
				}
			} else {
				numer = c+e-b-d;
				if (numer <= 0) {ls=0.0; lt=1.0; point=triangles[j].c;}
				else {
					denom = a-2*b+c;
					if (numer >= denom) {ls = 1.0; lt = 0.0; point=triangles[j].b;}
					else {ls = numer/denom; lt = 1.0-ls; onEdge=3;}
				}
			}
		}
		
		C = P1+ls*E0+lt*E1;
		D = C-X;
		est = mag(D);

		if (onEdge &&
				(((onEdge == 1) &&
					((edge[0] == triangles[j].a && edge[1] == triangles[j].b) ||
					 (edge[1] == triangles[j].a && edge[0] == triangles[j].b))) ||
				 ((onEdge == 2) &&
					((edge[0] == triangles[j].a && edge[1] == triangles[j].c) ||
					 (edge[1] == triangles[j].a && edge[0] == triangles[j].c))) ||
				 ((onEdge == 3) &&
					((edge[0] == triangles[j].b && edge[1] == triangles[j].c) ||
					 (edge[1] == triangles[j].b && edge[0] == triangles[j].c))))) {

			BfastVector3 fn = faceNormals[j];
			normalize(fn);
			BfastReal ndp = dot(D,fn);
			if (fabs(ndp) > fabs(dp)) {
				face = j;
				s = ls;
				t = lt;
				dp = ndp;
				sgn = -sign(dp);
			} 
		} else if (est < phi) {
			if (onEdge) {
				phi = est;
				face = j;
				s = ls;
				t = lt;
				BfastVector3 fn = faceNormals[j];
				normalize(fn);
				dp = dot(D,fn);
				sgn = -sign(dp);
				if (onEdge == 1) {edge[0] = triangles[j].a; edge[1] = triangles[j].b;}
				else if (onEdge == 2) {edge[0] = triangles[j].a; edge[1] = triangles[j].c;}
				else if (onEdge == 3) {edge[0] = triangles[j].b; edge[1] = triangles[j].c;}
			} else if (point != -1) {
				phi = est;
				face = j;
				s = ls;
				t = lt;
				sgn = -sign(dot(D, psuedoNormals[point]));
				dp = 0.0;
				edge[0] = edge[1] = -1;
			} else {
				phi = est;
				face = j;
				s = ls;
				t = lt;
				sgn =  -sign(orient3d(&(meshPts[triangles[j].a][0]), 
															&(meshPts[triangles[j].b][0]),
															&(meshPts[triangles[j].c][0]), 
															&(X[0])));
				edge[0] = edge[1] = -1;
				dp = 0;
			}
		}
	}
	if (sgn == 0) return phi;
	return sgn*phi;
}



BfastReal SlcSurface::interpolate(DtCell *c, const BfastVector3 &x) {
	//BfastReal ret;
	BfastVector3 y = x-tree->pts[c->vertices[0]];
	BfastVector3 z = tree->pts[c->vertices[7]]-tree->pts[c->vertices[0]];
	BfastReal w[3] = {y[0]/z[0], y[1]/z[1], y[2]/z[2]};
	return (BfastReal)
		(1.0-w[0])*(1.0-w[1])*(1.0-w[2])*(tree->phi[c->vertices[0]])+
		(1.0-w[0])*(1.0-w[1])*(    w[2])*(tree->phi[c->vertices[1]])+
		(1.0-w[0])*(    w[1])*(1.0-w[2])*(tree->phi[c->vertices[2]])+
		(1.0-w[0])*(    w[1])*(    w[2])*(tree->phi[c->vertices[3]])+
		(    w[0])*(1.0-w[1])*(1.0-w[2])*(tree->phi[c->vertices[4]])+
		(    w[0])*(1.0-w[1])*(    w[2])*(tree->phi[c->vertices[5]])+
		(    w[0])*(    w[1])*(1.0-w[2])*(tree->phi[c->vertices[6]])+
		(    w[0])*(    w[1])*(    w[2])*(tree->phi[c->vertices[7]]);
}



BfastReal SlcSurface::eval(const BfastVector3 &x, bool interpolate) {
	BfastReal y[3] = {x[0], x[1], x[2]}, s = 0.0, t = 0.0;
	int face = -1;
	return eval(y,s,t,face,interpolate);
}

BfastReal SlcSurface::eval(const BfastVector3 &x, BfastReal &s, BfastReal &t, int &face, bool interpolate) {
	BfastReal y[3] = {x[0], x[1], x[2]};
	return eval(y,s,t,face,interpolate);
}

BfastReal SlcSurface::eval(const BfastReal *x, bool interpolate) {
	BfastReal s = 0.0, t = 0.0;
	int face = -1;
	return eval(x,s,t,face,interpolate);
}

BfastReal SlcSurface::eval(const BfastReal *x, BfastReal &s, BfastReal &t, int &face, bool interpolate) {
	BfastReal ret, tmp;
	//if (!tree->cells[0].contains(tree, x)) return BFAST_REAL_MAX;
	DtCell *c = tree->findCell(x);
	if (c->level == tree->max_level && !interpolate) ret = exacteval(c,x,s,t,face);
	else ret = this->interpolate(c,x);
	if (minFunc && (tmp = minFunc(x, minFuncData)) < ret) ret = tmp;
	if (maxFunc && (tmp = maxFunc(x, maxFuncData)) > ret) ret = tmp;
	return ret;
}


BfastReal SlcSurface::creval(const BfastVector3 &x) {
	BfastReal y[3] = {x[0], x[1], x[2]};
	return creval(y);
}

BfastReal SlcSurface::creval(const BfastReal *x) {
	int i,l,m,n;
  BfastReal ret=0.0, tmp;
	DtCell *c = tree->findCell(x);
	BfastReal h = tree->pts[c->vertices[7]][0]-tree->pts[c->vertices[0]][0];
	int ih = tree->ipts[c->vertices[7]][0]-tree->ipts[c->vertices[0]][0];
  BfastVector3 d = (x-tree->pts[c->vertices[0]])/h;
  BfastReal w[3][4];
  BfastReal u;

  for (l=0; l<3; l++) {
    u = d[l];
    w[l][0] = 0.5*(-u*u*u+2*u*u-u); 
    w[l][1] = 0.5*(3*u*u*u-5*u*u+2); 
    w[l][2] = 0.5*(-3*u*u*u+4*u*u+u); 
    w[l][3] = 0.5*(u*u*u-u*u); 
  }

  for (n=-1; n<3; n++) 
    for (m=-1; m<3; m++)
      for (l=-1; l<3; l++) {
				i = bfastHash(tree->ipts[c->vertices[0]][0]+l*ih,tree->ipts[c->vertices[0]][1]+m*ih,
											tree->ipts[c->vertices[0]][2]+n*ih);
				if (tree->vertMap.count(i)) 
					ret+=w[0][l+1]*w[1][m+1]*w[2][n+1]*tree->phi[tree->vertMap[i]];
				else 
					ret+=w[0][l+1]*w[1][m+1]*w[2][n+1]*
						eval(BfastVector3(tree->pts[c->vertices[0]][0]+l*h,tree->pts[c->vertices[0]][1]+m*h,
															tree->pts[c->vertices[0]][2]+n*h),true);
			}

	if (minFunc && (tmp = minFunc(x, minFuncData)) < ret) ret = tmp;
	if (maxFunc && (tmp = maxFunc(x, maxFuncData)) > ret) ret = tmp;
	return ret;
}


BfastVector3 SlcSurface::grad(const BfastVector3 &x) {
	BfastVector3 y(x);
	return grad(&y[0]);
}

BfastVector3 SlcSurface::grad(const BfastReal *x) {
  //int i;
	DtCell *c = tree->findCell(x);
	BfastVector3 g;

	BfastVector3 y = x-tree->pts[c->vertices[0]];
	BfastVector3 z = tree->pts[c->vertices[7]]-tree->pts[c->vertices[0]];
	BfastReal w[3] = {y[0]/z[0], y[1]/z[1], y[2]/z[2]};
	
	g[0] = (1.0-w[1])*(1.0-w[2])*(tree->phi[c->vertices[4]]-tree->phi[c->vertices[0]])+
		(1.0-w[1])*(w[2])*(tree->phi[c->vertices[5]]-tree->phi[c->vertices[1]])+
		(w[1])*(1.0-w[2])*(tree->phi[c->vertices[6]]-tree->phi[c->vertices[2]])+
		(w[1])*(w[2])*(tree->phi[c->vertices[7]]-tree->phi[c->vertices[3]]);
	g[1] = (1.0-w[0])*(1.0-w[2])*(tree->phi[c->vertices[2]]-tree->phi[c->vertices[0]])+
		(1.0-w[0])*(w[2])*(tree->phi[c->vertices[3]]-tree->phi[c->vertices[1]])+
		(w[0])*(1.0-w[2])*(tree->phi[c->vertices[6]]-tree->phi[c->vertices[4]])+
		(w[0])*(w[2])*(tree->phi[c->vertices[7]]-tree->phi[c->vertices[5]]);
	g[2] = (1.0-w[0])*(1.0-w[1])*(tree->phi[c->vertices[1]]-tree->phi[c->vertices[0]])+
		(1.0-w[0])*(w[1])*(tree->phi[c->vertices[3]]-tree->phi[c->vertices[2]])+
		(w[0])*(1.0-w[1])*(tree->phi[c->vertices[5]]-tree->phi[c->vertices[4]])+
		(w[0])*(w[1])*(tree->phi[c->vertices[7]]-tree->phi[c->vertices[6]]);
	g/=(z[0]);
	return g;
}


istream &operator>>(std::istream &strm, SlcSurface &s) {
  //unsigned int i,newNum = 0;
  ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatStr("[",strm);
  strm >> *(s.tree);
  eatChar(',',strm);
  strm >> s.meshPts;
  eatChar(',',strm);
  strm >> s.triangles;
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}

ostream &operator<<(ostream &strm,const SlcSurface &s) {
  strm << "[";
  strm << *(s.tree);
  strm << ",";
  strm << s.meshPts;
  strm << ",";
  strm << s.triangles;
  strm << "]";
  return strm;
}

void SlcSurface::read(std::ifstream &in) {
	int version;
	in.read((char*)&version, sizeof(int));
	if (in.fail()) {
		cerr<<"can't read from file"<<endl;
		exit(-1);
	}
	if (version == 1) {
		tree->read(in);
		meshPts.read(in);
		triangles.read(in);
	} else {		
		cerr<<"incompatible bdt file version="<<version<<endl;
	}
	computeNormals();
	computePsuedoNormals();
}

void SlcSurface::write(std::ofstream &out) {
	int version = 1;
	out.write((char*)&version, sizeof(int));
  tree->write(out);
  meshPts.write(out);
  triangles.write(out);
}

void SlcSurface::bdtDump(const char *fname) {
	ofstream out(fname, ios::out | ios::binary);
	write(out);
}

void SlcSurface::bdtRead(const char *fname) {
	ifstream in(fname, ios::in | ios::binary);
	read(in);
}

void SlcSurface::objDump(const char *fname) {
	ofstream out(fname, ios::out);
	unsigned int i;
	for (i=0; i<meshPts.numVecs(); i++)
		out<<"v "<<meshPts[i][0]<<" "<<meshPts[i][1]<<" "<<meshPts[i][2]<<endl;
	for (i=0; i<triangles.numTriangles(); i++)
		out<<"f "<<triangles[i].a+1<<" "<<triangles[i].b+1<<" "<<triangles[i].c+1<<endl;
}

void SlcSurface::bvtDump(const char *fname) {	
	ofstream fout (fname, ios::out | ios::binary);
	int version = 1;
	fout.write((char*)&version, sizeof(int));
  meshPts.write(fout);
  triangles.write(fout);
}

void SlcSurface::bvtRead(const char *fname) {
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
}

void SlcSurface::ribDump(const char *fname) {
	ofstream out(fname, ios::out);
	unsigned int i;
	out<<"PointsPolygons [";
	for (i=0; i<triangles.numTriangles(); i++)
		out<<"3 ";
	out<<"] [";
	for (i=0; i<triangles.numTriangles(); i++)
		out<<triangles[i].a<<" "<<triangles[i].b<<" "<<triangles[i].c<<" ";
	out<<"]\n";

	out<<"\"P\" [";
	for (i=0; i<meshPts.numVecs(); i++)
		out<<meshPts[i][0]<<" "<<meshPts[i][1]<<" "<<meshPts[i][2]<<" ";
	out<<"]\n\"N\" [";
	for (i=0; i<normals.numVecs(); i++)
		out<<normals[i][0]<<" "<<normals[i][1]<<" "<<normals[i][2]<<" ";
	out<<"]\n\n";
#if 0
	out<<"\"Cs\" [";
	
	BfastVector3 c(0.35, 0.35, 0.35);	
	for (i=0; i<meshPts.numVecs(); i++) {
		BfastReal dist = mag(meshPts[i]-c)-0.15;
#if 0
		BfastReal h,s,v,r,g,b;
		h = 100000*dist+180;
		s = 0.22;
		v = 0.9;

		//cout<<h<<endl;

		if (h < 0) h = 0;
		if (h > 360) h = 360;
		
		h/=60.0;
		int i = (int)floor(h);
		BfastReal f = h-i;
		BfastReal p = v*(1.0-s);
		BfastReal q = v*(1.0 - (s*f));
		BfastReal t = v*(1.0-(s*(1.0-f)));
		//cout<<h<<" "<<s<<" "<<v<<" "<<dist<<" "<<i<<" "<<f<<" "<<p<<" "<<q<<" "<<t<<endl;
		switch(i) {
	  case 0: r=v; g=t; b=p; break;
	  case 1: r=q; g=v; b=p; break;
	  case 2: r=p; g=v; b=t; break;
	  case 3: r=p; g=q; b=v; break;
	  case 4: r=t; g=p; b=v; break;
	  case 5: r=v; g=p; b=q; break;
		}
		out<<r<<" "<<g<<" "<<b<<" ";
#else		
		//BfastVector3 c1(0.0, 0.5, 0.0), c2(1.0, 1.0, 1.0), c3(0.75,0.0,0.0);
		//BfastVector3 c1(0.53, 0.57, 0.63), c2(0.7,0.7,0.9), c3(0.54,0.48,0.45);
		//BfastVector3 c1(0.97, 1.0, 0.62), c2(0.3,0.3,0.4), c3(0.54,0.48,0.45);
		//BfastVector3 c1(0.97, 1.0, 0.62), c2(0.3,0.3,0.4), c3(0.85,0.75,0.65);
		// 130 127 235, 214 216 162
		//BfastVector3 c1(0.97, 1.0, 0.5), c2(1.0,1.0,1.0), c3(0.73,0.67,0.64);
		BfastVector3 c1(0.51, 0.5, 0.92), c2(1.0,1.0,1.0), c3(1.0,1.0,0.0);
		BfastVector3 color;
		dist *= 333;
		if (dist < 0) {
			if (dist < -1) dist = -1;
			color = (-dist)*c1+(1+dist)*c2;
		} else {
			if (dist > 1) dist = 1;
			color = dist*c3+(1-dist)*c2;
		}
		if (color[0] < 0) color[0] = 0;
		if (color[1] < 0) color[1] = 0;
		if (color[2] < 0) color[2] = 0;
		if (color[0] > 1) color[0] = 1;
		if (color[1] > 1) color[1] = 1;
		if (color[2] > 1) color[2] = 1;
		out<<color[0]<<" "<<color[1]<<" "<<color[2]<<" ";
#endif
	}
	out<<"]\n\n";
#endif
}

void readObjFile(const char *fname, BfastVecList &pts, BfastTriList &triangles) {
	ifstream in(fname, ios::in);
	char c[1000];
	int vt = 0;
	int vn = 0;
	int i = 0;
	BfastVector3 lc(BFAST_REAL_MAX), uc(-BFAST_REAL_MAX);

        std::cout << "SLC: readObjFile "<<fname<<std::endl;

	while ( (in >> c) && (strcmp(c,"v") != 0) );

  if ( strcmp(c,"v") == 0 ) {
		i=0;
    while ( strcmp(c,"v") == 0 ) {
			pts.setNumVecs(i+1,1,0);
      in >> pts[i][0];
      in >> pts[i][1];
      in >> pts[i][2];

			if (pts[i][0] < lc[0]) lc[0] = pts[i][0];
			if (pts[i][1] < lc[1]) lc[1] = pts[i][1];
			if (pts[i][2] < lc[2]) lc[2] = pts[i][2];
			if (pts[i][0] > uc[0]) uc[0] = pts[i][0];
			if (pts[i][1] > uc[1]) uc[1] = pts[i][1];
			if (pts[i][2] > uc[2]) uc[2] = pts[i][2];

      in >> c;
			i++;
    }
	}

    std::cout << "SLC: "<<i<<" vertices read, bbox="<<lc<<" - "<<uc<<std::endl;

  while ( (strcmp(c,"f") != 0) && (strcmp(c,"vt") != 0) && (strcmp(c,"vn") != 0) && ( in >> c) );

  if ( strcmp(c,"vt") == 0 ) {
    vt = 1;
    do {
      BfastReal temp;
			in >> temp;
      in >> temp;
      in >> c;
    } while ( strcmp(c,"vt") == 0 );
  }

  while ( (strcmp(c,"f") != 0) && (strcmp(c,"vn") != 0) && ( in >> c) );

  if ( strcmp(c,"vn") == 0 ) {
    vn = 1;
    while ( strcmp(c,"vn") == 0 ) {
			BfastReal temp;
      in >> temp;
      in >> temp;
      in >> temp;
      in >> c;
    }
  }

  while ( (strcmp(c,"f") != 0) && ( in >> c) );
  if ( strcmp(c,"f") == 0 ) {
		i=0;
    do {
      if ( strcmp(c,"f") != 0 )
				break;

			triangles.setNumTriangles(i+1,1,0);
      int temp;
      char ct;

      in >> triangles[i].a;
      if ( vt ) { in >> ct; in >> temp; }
      if ( vn ) { in >> ct; in >> temp; }
      
      in >> triangles[i].b;
      if ( vt ) { in >> ct; in >> temp; }
      if ( vn ) { in >> ct; in >> temp; }
      
      in >> triangles[i].c;
      if ( vt ) { in >> ct; in >> temp; }
      if ( vn ) { in >> ct; in >> temp; }
			triangles[i].a -= 1;
			triangles[i].b -= 1;
			triangles[i].c -= 1;
			i++;
    } while ( in >> c );
  }
}


#ifdef INVENTOR
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoNormalBinding.h>
SoSeparator *SlcSurface::ivTree() {
	SoSeparator *mesh = new SoSeparator();
	mesh->ref();
	
	SoNormal *sonormal = new SoNormal();
	mesh->addChild(sonormal);
	SoNormalBinding *normalBinding = new SoNormalBinding();
	mesh->addChild(normalBinding);
	normalBinding->value = SoNormalBinding::PER_VERTEX_INDEXED;
	
	for(int i=0; i<normals.numVecs(); i++) 
		sonormal->vector.set1Value(i,normals[i][0],normals[i][1],normals[i][2]);

	mesh->addChild(meshPts.coord());
	mesh->addChild(triangles.faceSet());
	mesh->unrefNoDelete();
	return mesh;
}

void SlcSurface::ivDump(const char *fname) {
	SoSeparator *iv = ivTree();
	iv->ref();
	bfastWriteIVFile(fname, iv);
	iv->unref();
}

#endif

void SlcSurface::contourTree(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), void *evalPhiData) {
	unsigned int i;
	//bool apos, bpos, cpos, dpos;	
	BfastIndexList cubes;
	DtCell *c;
	edgeList.clear();

	// create list of cubes to look at
	c = &(tree->cells[0]);
	for (i=0; i<tree->cells.numCells(); i++, c++) {
		if (c->level != tree->max_level) continue;
		int foo = sign(tree->phi[c->vertices[0]]);
		if ((foo != sign(tree->phi[c->vertices[1]])) ||
				(foo != sign(tree->phi[c->vertices[2]])) ||
				(foo != sign(tree->phi[c->vertices[3]])) ||
				(foo != sign(tree->phi[c->vertices[4]])) ||
				(foo != sign(tree->phi[c->vertices[5]])) ||
				(foo != sign(tree->phi[c->vertices[6]])) ||
				(foo != sign(tree->phi[c->vertices[7]])))
				cubes.add(i);
	}

	for (i=0; i<cubes.getNum(); i++) {
		c = &(tree->cells[cubes[i]]);
		BfastVector3 &x0 = tree->pts[c->vertices[0]];
		BfastVector3 &x1 = tree->pts[c->vertices[1]];
		BfastVector3 &x2 = tree->pts[c->vertices[2]];
		BfastVector3 &x3 = tree->pts[c->vertices[3]];
		BfastVector3 &x4 = tree->pts[c->vertices[4]];
		BfastVector3 &x5 = tree->pts[c->vertices[5]];
		BfastVector3 &x6 = tree->pts[c->vertices[6]];
		BfastVector3 &x7 = tree->pts[c->vertices[7]];

		dotet(evalPhi,c,c->vertices[0],c->vertices[2],c->vertices[4],c->vertices[1],
					x0,x2,x4,x1, evalPhiData);
		dotet(evalPhi,c,c->vertices[6],c->vertices[2],c->vertices[1],c->vertices[4],
					x6,x2,x1,x4, evalPhiData);
		dotet(evalPhi,c,c->vertices[6],c->vertices[2],c->vertices[3],c->vertices[1],
					x6,x2,x3,x1, evalPhiData);
		dotet(evalPhi,c,c->vertices[6],c->vertices[4],c->vertices[1],c->vertices[5],
					x6,x4,x1,x5, evalPhiData);
		dotet(evalPhi,c,c->vertices[6],c->vertices[1],c->vertices[3],c->vertices[5],
					x6,x1,x3,x5, evalPhiData);
		dotet(evalPhi,c,c->vertices[6],c->vertices[3],c->vertices[7],c->vertices[5],
					x6,x3,x7,x5, evalPhiData);
	}

	edgeList.clear();

	triangles.setNumTriangles(triangles.numTriangles(),1,1);
	meshPts.setNumVecs(meshPts.numVecs(),1,1);
	computeNormals();
	computePsuedoNormals();
}

int SlcSurface::vertid(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), int a, int b, BfastVector3 x, 
											 BfastVector3 y, bool apos, bool bpos, BfastReal aphi, BfastReal bphi, void *evalPhiData) {
	//int i;
	BfastReal TOL = 1e-10;
	int MAXITER = 100;
	BfastReal p,sum;
	bool cpos;
	int iter = 0;
	unsigned long long ei;
	int vi;
	BfastVector3 z,v,zprime;

	BfastVector3 ox(x), oy(y);
	BfastReal odist = sqrMag(y-x);

	if (a > b) { 
		int ti;
		bool tb;
		BfastReal td;
		BfastVector3 tv;
		ti = a; a = b; b = ti; 
		tv = x; x = y; y = tv;
		tb = apos; apos = bpos; bpos = tb;
		td = aphi; aphi = bphi; bphi = td;
	}
	
	ei = a;
	ei = ei << 32;
	ei+=b;
	if (edgeList.count(ei))
		return edgeList[ei];

	sum = fabs(aphi-bphi);
	if (sum < TOL) z = 0.5*(x+y);
	else z = (fabs(bphi)/sum)*x+(fabs(aphi)/sum)*y;
	if (evalPhi)
	while (fabs(p=evalPhi(z,evalPhiData)) > TOL && iter++ < MAXITER) {
		if ((cpos = (p > 0.0)) == apos) {
			x = z;
			aphi = p;
		} else {
			y = z;
			bphi = p;
		}
		sum = fabs(aphi-bphi);
		z = (fabs(bphi)/sum)*x+(fabs(aphi)/sum)*y;
	}

	if (sqrMag(ox-z) < 0.000001*odist)
		z = 0.999*ox+0.001*oy;

	if (sqrMag(oy-z) < 0.000001*odist)
		z = 0.001*ox+0.999*oy;

	vi = meshPts.numVecs();
	edgeList[ei] = vi;
	meshPts.setNumVecs(meshPts.numVecs()+1,1,0);
	meshPts[vi] = z;

	return vi;
}

void SlcSurface::dotet(BfastReal (*evalPhi)(const BfastVector3 &x, void *data), DtCell *cell, 
									 int a, int b, int c, int d,
											 BfastVector3 w, BfastVector3 x, BfastVector3 y, BfastVector3 z, void *evalPhiData) {
	bool apos, bpos, cpos, dpos;
	BfastReal aphi, bphi, cphi, dphi;
	int index=0, e1=0, e2=0, e3=0, e4=0, e5=0, e6=0;
	BfastVector3 foo;
	if (apos = ((aphi=tree->phi[a]) > 0.0)) index += 8;
	if (bpos = ((bphi=tree->phi[b]) > 0.0)) index += 4;
	if (cpos = ((cphi=tree->phi[c]) > 0.0)) index += 2;
	if (dpos = ((dphi=tree->phi[d]) > 0.0)) index += 1;

	if (apos != bpos) e1 = vertid(evalPhi,a,b,w,x,apos,bpos,aphi,bphi,evalPhiData);
	if (apos != cpos) e2 = vertid(evalPhi,a,c,w,y,apos,cpos,aphi,cphi,evalPhiData);
	if (apos != dpos) e3 = vertid(evalPhi,a,d,w,z,apos,dpos,aphi,dphi,evalPhiData);
	if (bpos != cpos) e4 = vertid(evalPhi,b,c,x,y,bpos,cpos,bphi,cphi,evalPhiData);
	if (bpos != dpos) e5 = vertid(evalPhi,b,d,x,z,bpos,dpos,bphi,dphi,evalPhiData);
	if (cpos != dpos) e6 = vertid(evalPhi,c,d,y,z,cpos,dpos,cphi,dphi,evalPhiData);
	
	switch (index) {
	case 1: cell->triangles.add(triangles.add(e5,e3,e6)); break;
	case 2: cell->triangles.add(triangles.add(e2,e4,e6)); break;
	case 3: cell->triangles.add(triangles.add(e3,e4,e5)); 
		cell->triangles.add(triangles.add(e3,e2,e4)); break;
	case 4: cell->triangles.add(triangles.add(e1,e5,e4)); break;
	case 5: cell->triangles.add(triangles.add(e3,e4,e1)); 
		cell->triangles.add(triangles.add(e3,e6,e4)); break;
	case 6: cell->triangles.add(triangles.add(e1,e6,e2)); 
		cell->triangles.add(triangles.add(e1,e5,e6)); break;
	case 7: cell->triangles.add(triangles.add(e1,e3,e2)); break;
	case 8: cell->triangles.add(triangles.add(e1,e2,e3)); break;
	case 9: cell->triangles.add(triangles.add(e1,e6,e5)); 
		cell->triangles.add(triangles.add(e1,e2,e6)); break;
	case 10: cell->triangles.add(triangles.add(e1,e6,e3)); 
		cell->triangles.add(triangles.add(e1,e4,e6)); break;
	case 11: cell->triangles.add(triangles.add(e1,e4,e5)); break;
	case 12: cell->triangles.add(triangles.add(e3,e4,e2)); 
		cell->triangles.add(triangles.add(e3,e5,e4)); break;
	case 13: cell->triangles.add(triangles.add(e6,e4,e2)); break;
	case 14: cell->triangles.add(triangles.add(e5,e6,e3)); break;
	}
}

void SlcSurface::computeNormals() {
	unsigned int i;
	faceNormals.setNumVecs(triangles.numTriangles());
	normals.setNumVecs(meshPts.numVecs()); 
	for (i=0; i<normals.numVecs(); i++)
		normals[i] = 0.0;

	for (i=0; i<faceNormals.numVecs(); i++) {
		faceNormals[i] = cross(meshPts[triangles[i].b]-meshPts[triangles[i].a], 
													 meshPts[triangles[i].c]-meshPts[triangles[i].a]);
		normalize(faceNormals[i]);
		normals[triangles[i].a]+=faceNormals[i];
		normals[triangles[i].b]+=faceNormals[i];
		normals[triangles[i].c]+=faceNormals[i];
	}

	for (i=0; i<normals.numVecs(); i++)
		normalize(normals[i]);
}

void SlcSurface::computePsuedoNormals() {
	//if (psuedoNormals.numVecs()) return;
	unsigned int i;
	BfastVector3 V1, V2, V3, fn;
	BfastReal tmp;
	psuedoNormals.setNumVecs(meshPts.numVecs()); //this zeros all the vectors
	for (i=0; i<psuedoNormals.numVecs(); i++) {
		psuedoNormals[i] = 0.0;
	}
	for (i=0; i<faceNormals.numVecs(); i++) {
		fn = faceNormals[i];
		V1 = meshPts[triangles[i].b]-meshPts[triangles[i].a];
		V2 = meshPts[triangles[i].c]-meshPts[triangles[i].b];
		V3 = meshPts[triangles[i].a]-meshPts[triangles[i].c];
		normalize(fn);
		normalize(V1);
		normalize(V2);
		normalize(V3);

		tmp = dot(V1,-V3);
		if (tmp < 1.0)
			psuedoNormals[triangles[i].a]+=acos(tmp)*fn;
		tmp = dot(V2,-V1);
		if (tmp < 1.0)
			psuedoNormals[triangles[i].b]+=acos(tmp)*fn;
		tmp = dot(V3,-V2);
		if (tmp < 1.0)
			psuedoNormals[triangles[i].c]+=acos(tmp)*fn;
	}

	for (i=0; i<psuedoNormals.numVecs(); i++) {
		normalize(psuedoNormals[i]);
	}
}

#ifdef INVENTOR
int bfastWriteIVFile(const char *filename, SoNode *node) {
  // Open the output file
  SoOutput mySceneOutput;
  if (strcmp(filename,"-")==0) {
    mySceneOutput.setFilePointer(stdout);
  } else if (strcmp(filename,"--")==0) {
    mySceneOutput.setFilePointer(stderr);
  } else if (!mySceneOutput.openFile(filename)) {
    return false;
  }

	mySceneOutput.setBinary(true);

  // Read the whole file into the database
  SoWriteAction myWriter(&mySceneOutput);
  myWriter.apply(node);

  if ((strcmp(filename,"-")!=0) && (strcmp(filename,"--")!=0)) {
    mySceneOutput.closeFile();
  }

  return true;
}
#endif

///////////////////////////////////////////////////////////////////
// Code for fast marching method
///////////////////////////////////////////////////////////////////

struct eqint {
  bool operator()(const int a, const int b) const {
    return (a == b);
  }
};

////////////////////////////////////////////////////////////////////////
// DtFMMHeap (used for fast marching method)
////////////////////////////////////////////////////////////////////////
DtFMMHeap::DtFMMHeap(int dim, int dim2) {
  _numEntries = 0;
  _numMalloc = 0;
  _entries = NULL;
  mymap.clear();
  this->dim = dim;
  this->dim2 = dim2;
}

DtFMMHeap::~DtFMMHeap() {
  if (_entries) delete [] _entries;
  mymap.clear();
}

void DtFMMHeap::swap(int i, int j) {
  DtFMMHeapEntry tmp = _entries[i];
  _entries[i] = _entries[j];
  _entries[j] = tmp;
  mymap[bfastHash(_entries[i].i,_entries[i].j,_entries[i].k)] = i;
  mymap[bfastHash(_entries[j].i,_entries[j].j,_entries[j].k)] = j;
}

DtFMMHeapEntry *DtFMMHeap::findEntry(int x, int y, int z) {
  int foo = bfastHash(x,y,z);
  if (mymap.count(foo))
    return &(_entries[mymap[foo]]);
  else
    return NULL;
}

void DtFMMHeap::addToHeap(int x, int y, int z, BfastReal phi) {
  DtFMMHeapEntry *entry = _entries;
  int i, foo = bfastHash(x,y,z);

  if (mymap.count(foo)) {
    entry = &(_entries[i=mymap[foo]]);
    entry->phi = phi;
    while (i>0 && fabs(phi) < fabs(_entries[(i-1)/2].phi)) {
      swap(i,(i-1)/2);
      i = (i-1)/2;
    }
    while (i*2+1 < _numEntries) {
      if (fabs(_entries[i*2+1].phi) < fabs(phi))
				if (i*2+2 < _numEntries && fabs(_entries[i*2+2].phi) < fabs(phi))
					if (fabs(_entries[i*2+1].phi) < fabs(_entries[i*2+2].phi)) {
						swap(i,i*2+1);
						i=i*2+1;
					} else {
						swap(i,i*2+2);
						i=i*2+2;
					}
				else {
					swap(i,i*2+1);
					i = i*2+1;
				}
      else if (i*2+2 < _numEntries && fabs(_entries[i*2+2].phi) < fabs(phi)) {
				swap(i,i*2+2);
				i=i*2+2;
      } else break;
    }
    return;
  }
  setNumEntries(_numEntries+1,1,0);
  entry = _entries+_numEntries-1;
  entry->i = x;
  entry->j = y;
  entry->k = z;
  entry->phi = phi;
  mymap[foo] = _numEntries-1;
  i = _numEntries-1;
  while(i>0 && fabs(phi)<fabs(_entries[(i-1)/2].phi)) {
    swap(i,(i-1)/2);
    i=(i-1)/2;
  }
}

DtFMMHeapEntry DtFMMHeap::removeFromHeap(){
  DtFMMHeapEntry ret = _entries[0];
  DtFMMHeapEntry e = _entries[numEntries()-1];
  swap(0,numEntries()-1);
  setNumEntries(_numEntries-1,1,0);
  int i=0;
  while (i*2+1 < _numEntries) {
    if (fabs(_entries[i*2+1].phi) < fabs(e.phi))
      if (i*2+2 < _numEntries && fabs(_entries[i*2+2].phi) < fabs(e.phi))
				if (fabs(_entries[i*2+1].phi) < fabs(_entries[i*2+2].phi)) {
					swap(i,i*2+1);
					i = i*2+1;
				} else {
					swap(i,i*2+2);
					i = i*2+2;
				}
      else {
				swap(i,i*2+1);
				i = i*2+1;
      }
    else if (i*2+2 < _numEntries && fabs(_entries[i*2+2].phi) < fabs(e.phi)) {
      swap(i,i*2+2);
      i=i*2+2;
    } else break;
  }
  i = bfastHash(ret.i,ret.j,ret.k);
  mymap.erase(i);
  return ret;
}

void DtFMMHeap::setNumEntries(unsigned int num, int copyOld, int exactSize) {
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
    unsigned int oldSize = _numEntries;
    DtFMMHeapEntry *oldList = _entries;
    _numEntries = num;
    _numMalloc = newMalloc;
    _entries = new DtFMMHeapEntry[_numMalloc];
    if (copyOld) {
      int l=(num>oldSize)?(oldSize):(num);
      for (int i=0; i<l; i++) {
				_entries[i] = oldList[i];
      }
    }
    delete [] oldList;
  } else {
    _numEntries = num;
  }
}

void SlcSurface::addToFMMHeap(BfastInt3 ipt, DtFMMHeap &heap, 
															short *levels, bool *accepted, BfastRealList &newPhi,
															int foo) {
	addToFMMHeap(ipt[0], ipt[1], ipt[2], heap, levels, accepted, newPhi, foo);
}

void SlcSurface::addToFMMHeap(int i, int j, int k, DtFMMHeap &heap, 
													short *levels, bool *accepted, BfastRealList &newPhi,
													int foo) {
  BfastReal p, phi1, phi2, phi3, q, a=0, b=0, c=0, x1, x2;
	int u, v, x, y, dim=tree->dim; //, dim2=tree->dim2;
  int d1, d2, d3;
  phi1=phi2=phi3=BFAST_REAL_MAX;
  d1=d2=d3=0;

	if (foo && heap.findEntry(i,j,k)) return;
	x = bfastHash(i,j,k);
	if (tree->vertMap.count(x)) u=tree->vertMap[x];
	else u=-1;
	int stride = (int) pow2(tree->max_level-levels[u]);
	BfastReal h = pow2(-levels[u])*(tree->uc[0]-tree->lc[0]);
	BfastReal h2 = h*h;

	v = -1;
	if (i-stride >= 0) {
		y = x-stride;
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	} 
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi1 = newPhi[v];
    d1 = 1;
  }
  
	v = -1;
	if (i+stride < dim) {
		y = x+stride;
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	}
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi1 = absmin2(phi1,newPhi[v]);
    d1 = 1;
  }

	v = -1;
	if (j-stride >= 0) {
		y = x-(stride<<10);
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	}
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi2 = newPhi[v];
    d2 = 1;
  } 

	v = -1;
	if (j+stride < dim) {
		y = x+(stride<<10);
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	}
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi2 = absmin2(phi2,newPhi[v]);
    d2 = 1;
  }

	v = -1;
	if (k-stride >= 0) {
		y = x-(stride<<20);
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	}
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi3 = newPhi[v];
    d3 = 1;
  } 
  
	v = -1;
	if (k+stride < dim) {
		y = x+(stride<<20);
		if (tree->vertMap.count(y)) v = tree->vertMap[y];
	}
  if ((v!=-1) && accepted[v] && (levels[u]<=levels[v])) {
    phi3 = absmin2(phi3,newPhi[v]);
    d3 = 1;
  }

  int done = 0;
  if (d1 && d2 && d3) {
    if ((((phi1-phi2)/h)*((phi1-phi2)/h)+((phi1-phi3)/h)*((phi1-phi3)/h) > 1.0) ||
				(((phi2-phi1)/h)*((phi2-phi1)/h)+((phi2-phi3)/h)*((phi2-phi3)/h) > 1.0) ||
				(((phi3-phi1)/h)*((phi3-phi1)/h)+((phi3-phi2)/h)*((phi3-phi2)/h) > 1.0)) {
      if (fabs(phi1) > fabs(phi2) && fabs(phi1) > fabs(phi3))
				d1 = 0;
      else if (phi2 > phi3)
				d2 = 0;
      else
				d3 = 0;
    } else {
      a = 1.0/(h2)+1.0/(h2)+1.0/(h2);
      b = -2.0*(phi1/h2+phi2/h2+phi3/h2);
      c = (phi1*phi1/h2)+(phi2*phi2/h2)+(phi3*phi3/h2)-1.0;
      done = 1;
    }
  }
  if (!done && d1 && d2) {
    if ((((phi1-phi2)/h)*((phi1-phi2)/h) > 1.0) ||
				(((phi2-phi1)/h)*((phi2-phi1)/h) > 1.0)) {
      if (fabs(phi1) > fabs(phi2))
				d1 = 0;
      else 
				d2 = 0;
    } else {
      a = 1.0/(h2)+1.0/(h2);
      b = -2.0*(phi1/h2+phi2/h2);
      c = (phi1*phi1/h2)+(phi2*phi2/h2)-1.0;
      done = 1;
    }
  }
  if (!done && d1 && d3) {
    if ((((phi1-phi3)/h)*((phi1-phi3)/h) > 1.0) ||
				(((phi3-phi1)/h)*((phi3-phi1)/h) > 1.0)) {
      if (fabs(phi1) > fabs(phi3))
				d1 = 0;
      else 
				d3 = 0;
    } else {
      a = 1.0/(h2)+1.0/(h2);
      b = -2.0*(phi1/h2+phi3/h2);
      c = (phi1*phi1/h2)+(phi3*phi3/h2)-1.0;
      done = 1;
    }
  }
  if (!done && d2 && d3) {
    if ((((phi2-phi3)/h)*((phi2-phi3)/h) > 1.0) ||
				(((phi3-phi2)/h)*((phi3-phi2)/h) > 1.0)) {
      if (fabs(phi2) > fabs(phi3))
				d2 = 0;
      else 
				d3 = 0;
    } else {
      a = 1.0/(h2)+1.0/(h2);
      b = -2.0*(phi2/h2+phi3/h2);
      c = (phi2*phi2/h2)+(phi3*phi3/h2)-1.0;
      done = 1;
    }
  }
  if (!done && d1) {
    a = 1.0/(h2);
    b = -2.0*(phi1/h2);
    c = (phi1*phi1/h2)-1.0;
    done = 1;
  }
  if (!done && d2) {
    a = 1.0/(h2);
    b = -2.0*(phi2/h2);
    c = (phi2*phi2/h2)-1.0;
    done = 1;
  }
  if (!done && d3) {
    a = 1.0/(h2);
    b = -2.0*(phi3/h2);
    c = (phi3*phi3/h2)-1.0;
    done = 1;
  }

  if (b >= 0) 
    q = -0.5*(b+sqrt(b*b-4*a*c));
  else
    q = 0.5*(-b+sqrt(b*b-4*a*c));
  x1 = q/a;
  x2 = c/q;

  BfastReal oldPhi = tree->phi[u];
	if (oldPhi == 0.0) 	p = absmax2(x1,x2);
	else {
		if (sign(oldPhi) != sign(x1))
			if (sign(oldPhi) != sign(x2)) 
				p = absmax2(x1,x2);
			else p = x2;
		else if (sign(oldPhi) != sign(x2))
			p = x1;
		else
			p = absmax2(x1,x2);
	}
	p = absmax2(x1,x2);
  heap.addToHeap(i, j, k, p);
}

// first order accurate fast marching method
void SlcSurface::redistance() {
	int i,j,k,l,x,y,u,v,stride;
	int dim=tree->dim, dim2=tree->dim2;
  DtFMMHeap heap(dim,dim2);
  DtFMMHeapEntry e;
  //int cand;
	
	BfastRealList newPhi;
	tree->phi.setNumValues(tree->pts.numVecs());
	newPhi.setNumValues(tree->phi.numValues());
    
	DtCell *c;
	BfastReal d;
	//int face;
	//BfastReal s,t;
	BfastReal *p;
	short *lev;
	short *min_levels = new short[tree->pts.numVecs()];
	short *max_levels = new short[tree->pts.numVecs()];
	bool *acc, *accepted = new bool[tree->pts.numVecs()];

	p = &(newPhi[0]);
	acc = accepted;
	for (i=0; i<(int)tree->pts.numVecs(); i++, p++, acc++) {
		*p = BFAST_REAL_MAX;
		*acc = false;
	}
	
	for (i=tree->cells.numCells()-1; i >= 0; i--) {
		c = &(tree->cells[i]);
		if (c->child[0] != -1 && !c->triangles.getNum())  
			for (j=0; j<8; j++) 
				c->triangles.add(tree->cells[c->child[j]].triangles);
		if (c->level != tree->max_level) continue;
 		if (c->triangles.getNum()) {
			BfastReal h = tree->pts[c->vertices[7]][0]-tree->pts[c->vertices[0]][0];
			for (j=0; j<8; j++) {
				v = c->vertices[j];
				if (builtFromTriangles) visitedTris.clear();
				pointToTri(c,&(tree->pts[v][0]),d);
				if (fabs(d) > h) continue;
				if (fabs(d) < fabs(newPhi[v]) && d!=BFAST_REAL_MAX) {
					newPhi[v] = d;
					accepted[v] = true;
				}
			}
		}
	}

	//c = &(tree->cells[tree->cells.numCells()-1]);
	//for (i=tree->cells.numCells()-1; i >= 0; i--, c--) {
	//if (c->triangles.getNum() == 0 && c->child[0] != -1) 
	//	c->child[0] = c->child[1] = c->child[2] = c->child[3] =
	//		c->child[4] = c->child[5] = c->child[6] = c->child[7] = -1;
	//}

	lev = max_levels;
	for (i=0; i<(int)tree->pts.numVecs(); i++, lev++) 
		*lev = -1;

	lev = min_levels;
	for (i=0; i<(int)tree->pts.numVecs(); i++, lev++) 
		*lev = -1;

	for (i=0; i<(int)tree->cells.numCells(); i++) {
		c = &(tree->cells[i]);
		//if (c->parent != -1 && tree->cells[c->parent].child[0] == -1) continue;
		for (j=0; j<8; j++) {
			v = c->vertices[j];
			max_levels[v] = c->level;
			if (min_levels[v] == -1) min_levels[v] = c->level;
		}
	}

  bool doDelete = true;
  bool done = false;
  int iter = 0;

  while (!done && iter < 2) {
    for (i=tree->cells.numCells()-1; i>=0; i--) {
      c = &(tree->cells[i]);
      bool add=false;
      for (j=0; j<8 && !add; j++) if (accepted[c->vertices[j]]) add = true;
      if (add) {
        if (c->level == max_levels[c->vertices[0]] && !accepted[c->vertices[0]] &&
            (accepted[c->vertices[1]] || accepted[c->vertices[2]] || accepted[c->vertices[4]]))
          addToFMMHeap(tree->ipts[c->vertices[0]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[1]] && !accepted[c->vertices[1]] &&
            (accepted[c->vertices[0]] || accepted[c->vertices[3]] || accepted[c->vertices[5]]))
          addToFMMHeap(tree->ipts[c->vertices[1]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[2]] && !accepted[c->vertices[2]] &&
            (accepted[c->vertices[3]] || accepted[c->vertices[0]] || accepted[c->vertices[6]]))
          addToFMMHeap(tree->ipts[c->vertices[2]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[3]] && !accepted[c->vertices[3]] &&
            (accepted[c->vertices[2]] || accepted[c->vertices[1]] || accepted[c->vertices[7]]))
          addToFMMHeap(tree->ipts[c->vertices[3]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[4]] && !accepted[c->vertices[4]] &&
            (accepted[c->vertices[5]] || accepted[c->vertices[6]] || accepted[c->vertices[0]]))
          addToFMMHeap(tree->ipts[c->vertices[4]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[5]] && !accepted[c->vertices[5]] &&
            (accepted[c->vertices[4]] || accepted[c->vertices[7]] || accepted[c->vertices[1]]))
          addToFMMHeap(tree->ipts[c->vertices[5]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[6]] && !accepted[c->vertices[6]] &&
            (accepted[c->vertices[7]] || accepted[c->vertices[4]] || accepted[c->vertices[2]]))
          addToFMMHeap(tree->ipts[c->vertices[6]], heap, max_levels, accepted, newPhi,1);

        if (c->level == max_levels[c->vertices[7]] && !accepted[c->vertices[7]] &&
            (accepted[c->vertices[6]] || accepted[c->vertices[5]] || accepted[c->vertices[3]]))
          addToFMMHeap(tree->ipts[c->vertices[7]], heap, max_levels, accepted, newPhi,1);
      }
    }

    while (heap.numEntries()) {
      e = heap.removeFromHeap();
      i=e.i, j=e.j, k=e.k;
      x = bfastHash(i,j,k);
      u = tree->vertMap[x];
      newPhi[u] = e.phi;
      accepted[u] = true;

      for (l=max_levels[u]; l>=min_levels[u]; l--) {
        stride = (int) pow2(tree->max_level-l);

        v = -1;
        if (i-stride >= 0) {
          y = x-stride;
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i-stride, j, k, heap, max_levels, accepted, newPhi,0);

        v = -1;
        if (i+stride < dim) {
          y = x+stride;
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i+stride, j, k, heap, max_levels, accepted, newPhi,0);

        v = -1;
        if (j-stride >= 0) {
          y = x-(stride<<10);
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i, j-stride, k, heap, max_levels, accepted, newPhi,0);

        v = -1;
        if (j+stride < dim) {
          y = x+(stride<<10);
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i, j+stride, k, heap, max_levels, accepted, newPhi,0);

        v = -1;
        if (k-stride >= 0) {
          y = x-(stride<<20);
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i, j, k-stride, heap, max_levels, accepted, newPhi,0);

        v = -1;
        if (k+stride < dim) {
          y = x+(stride<<20);
          if (tree->vertMap.count(y)) v = tree->vertMap[y];
        }
        if ((v!=-1) && !accepted[v] && (l==max_levels[v]))
          addToFMMHeap(i, j, k+stride, heap, max_levels, accepted, newPhi,0);
      }
    }

    doDelete = false;

    acc = accepted;
    for (i=0; i<(int)tree->phi.numValues(); i++, acc++) if (!(*acc)) doDelete = true;
    if (doDelete) {
#ifdef WIN32
      const int nc = tree->cells.numCells();
      bool *deleteCell = new bool[nc];
#else
      bool deleteCell[tree->cells.numCells()];
#endif

      acc = deleteCell;
      for (i=0; i<(int)tree->cells.numCells(); i++, acc++) *acc = false;
      for (i=tree->cells.numCells()-1; i >= 0; i--) {
        c = &(tree->cells[i]);
        doDelete = true;
        if (c->child[0] != -1) {
          for (j=0; j<8; j++) if (!deleteCell[c->child[j]]) doDelete = false;
          if (doDelete) {
						tree->cells[c->child[0]].parent = -1;
						tree->cells[c->child[1]].parent = -1;
						tree->cells[c->child[2]].parent = -1;
						tree->cells[c->child[3]].parent = -1;
						tree->cells[c->child[4]].parent = -1;
						tree->cells[c->child[5]].parent = -1;
						tree->cells[c->child[6]].parent = -1;
						tree->cells[c->child[7]].parent = -1;
            c->child[0] = c->child[1] = c->child[2] = c->child[3] =
              c->child[4] = c->child[5] = c->child[6] = c->child[7] = -1;
            max_levels[c->vertices[0]] = c->level;
            max_levels[c->vertices[1]] = c->level;
            max_levels[c->vertices[2]] = c->level;
            max_levels[c->vertices[3]] = c->level;
            max_levels[c->vertices[4]] = c->level;
            max_levels[c->vertices[5]] = c->level;
            max_levels[c->vertices[6]] = c->level;
            max_levels[c->vertices[7]] = c->level;
          }
        }
        doDelete = true;
        for (j=0; j<8; j++) if (accepted[c->vertices[j]]) doDelete = false;
        if (doDelete) {
          deleteCell[i] = true;
        }
      }
#ifdef WIN32
      delete [] deleteCell;
#endif
    } else done = true;
		iter++;
	}
						
	delete [] accepted;
	delete [] max_levels;
	delete [] min_levels;

	tree->phi = newPhi;
}

void SlcSurface::setMaxFunc(BfastReal (*maxFunc)(const BfastReal *x, void *data), void *data) {
	this->maxFunc = maxFunc;
	this->maxFuncData = data;
}

void SlcSurface::setMinFunc(BfastReal (*minFunc)(const BfastReal *x, void *data), void *data) {
	this->minFunc = minFunc;
	this->minFuncData = data;
}

void SlcSurface::rebuildTree() {
	unsigned int i,h;
	BfastInt3 *ipt;
	BfastVector3 *x;
	BfastReal *p;
	DtTree *newTree = new DtTree();
	ConcentricTripleSplitCriterion splitCriterion;
	splitCriterion.tree = tree;
	newTree->buildTree(tree->lc, tree->uc, tree->max_level, splitCriterion);

	newTree->phi.setNumValues(newTree->pts.numVecs());
	ipt = &(newTree->ipts[0]);
	x = &(newTree->pts[0]);
	p = &(newTree->phi[0]);
	for (i=0; i<newTree->pts.numVecs(); i++, p++, x++, ipt++) {
		h=bfastHash(*ipt);
		if (tree->vertMap.count(h)) (*p) = tree->phi[tree->vertMap[h]];
		else (*p) = eval(*x);
	}
	//delete tree;
	tree = newTree;
}			

}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: slcSurface.cpp,v $
// Revision 1.6  2006/06/16 21:07:14  adamb
// new version
//
// Revision 1.5  2006/01/15 01:43:51  adamb
// added rebuildTree
//
// Revision 1.4  2006/01/02 23:49:07  adamb
// return DBL_MAX when evaluating pts outside bounding box
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
// Revision 1.4  2005/07/20 02:35:11  adamb
// forced checkin of slc code *after* changes to hash function
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
