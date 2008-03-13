//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Setup
//   -- Code File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
//
// This is just a little program to setup an input file for the
// surface tracking code.
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


#include "slcSurface.h"
#include <ctime>
#include <sys/time.h>
#include <cstdlib>
#include <float.h>

using namespace std;
using namespace sofa::helper::slc;

double phi(const BfastVector3 &x, void* /*data*/) {
	//BfastVector3 c(100.5, 63.5, 63.5);
	//BfastVector3 c(25.5, 63.5, 63.5);
	// Bunny Spray?
	//BfastVector3 c(47, 40.5, 70);
	//double rad = 3;
	//double height = 3;

	BfastVector3 c(20, 48.5, 64);
	double rad = 5;
	double height = 5;
  double cylinder = sqrt((x[0]-c[0])*(x[0]-c[0])+(x[2]-c[2])*(x[2]-c[2]))-
		rad; 
	double planes = abs(x[1]-c[1])-height;
	double ret = max2(planes, cylinder);

	return ret;
} 
#if 0
double phi(const BfastVector3 &x, void* data) {
	//BfastVector3 c(25.5, 63.5, 63.5);
	BfastVector3 c(63.5, 63.5, 63.5);
	//BfastVector3 lc(0.0,0.0,0.0), uc(1.0,0.1,1.0);
	BfastVector3 lc(0.5,0.5,0.5), uc(126.5,40.0,126.5);
	//BfastVector3 lc(40.0,90.0,40.0), uc(87.0,120.0,87.0);
  double sphere = mag(x-c)-20; 
	//sphere = DBL_MAX;
	double m,n,o;
	m = absmin2(lc[0]-x[0],x[0]-uc[0]);
	n = absmin2(lc[1]-x[1],x[1]-uc[1]);
	o = absmin2(lc[2]-x[2],x[2]-uc[2]);
	double ret = min2(sphere, max3(m,n,o));
	if (ret == 0.0) return -DBL_MIN;
	return ret;
} 

#else
#if 0
double phi(const BfastVector3 &x, void* data) {
	BfastVector3 c(63.5, 63.5, 63.5);
	BfastVector3 lc(0.5,0.5,0.5), uc(126.5,40.0,126.5);

  double cylinder = -sqrt((x[0]-c[0])*(x[0]-c[0])+(x[2]-c[2])*(x[2]-c[2]))+60; 
	double m,n,o;
	m = absmin2(lc[0]-x[0],x[0]-uc[0]);
	n = absmin2(lc[1]-x[1],x[1]-uc[1]);
	o = absmin2(lc[2]-x[2],x[2]-uc[2]);
	double ret = max3(m,n,o);
	if (x[1] <= uc[1]) {
		ret = max2(cylinder, ret);
	}
	if (ret == 0.0) return -DBL_MIN;
	return ret;
} 

double phi1(const BfastVector3 &x, void* data) {
	BfastVector3 c(63.5, 63.5, 63.5);
	BfastVector3 lc(0.5,0.5,0.5), uc(126.5,40.0,126.5);

  double cylinder = sqrt((x[0]-c[0])*(x[0]-c[0])+(x[2]-c[2])*(x[2]-c[2]))-60; 
	double m,n,o;
	m = absmin2(lc[0]-x[0],x[0]-uc[0]);
	n = absmin2(lc[1]-x[1],x[1]-uc[1]);
	o = absmin2(lc[2]-x[2],x[2]-uc[2]);
	double ret = max3(m,n,o);
	ret = max2(cylinder, ret);
	if (ret == 0.0) return -DBL_MIN;
	return ret;
} 
#endif

#endif

int main (int argc, char *argv[]) {
	int i,j;
	bool OBJ = false;
	char objfname[80];

	if (argc != 5 && argc != 7) {
		cerr<<"Usage: "<<argv[0]<<"[outputfile] [domain's lower corner] [domain's upper corner] [maximum octree depth]"<<endl;
		exit(1);
	}

  for (i=0,j=0; i<argc; i++) {
    if (strcmp("-obj", argv[i]) == 0) {
      OBJ = true;
      strcpy(objfname, argv[++i]);
    } else argv[j++] = argv[i];
  }

	DtTree t;
	SlcSurface s(&t);
// 	char fname[80];

	if (OBJ) {
		BfastVecList meshPts;
		BfastTriList triangles;
		BfastVecList vertexNormals;
		BfastVecList faceNormals;

		cout<<"Reading obj file \'"<<objfname<<"\'"<<endl;
		readObjFile(objfname, meshPts, triangles);
		for (unsigned i=0; i<meshPts.numVecs(); i++) {
			//meshPts[i] *= 75;
			//meshPts[i][0] += 63.5;
			//meshPts[i][1] += 24.77705;
			//meshPts[i][2] += 63.5;
			meshPts[i] *= 150;
			meshPts[i][0] += 63.5;
			meshPts[i][1] += 49.5541;
			meshPts[i][2] += 63.5;
		}

		computeNormals(meshPts, triangles, vertexNormals, faceNormals);
	
		buildTree(s.tree, BfastVector3(atof(argv[2])), BfastVector3(atof(argv[3])), atoi(argv[4]),
							&meshPts, &triangles, &faceNormals);
		s.meshPts = meshPts;
		s.triangles = triangles;
		s.computeNormals();
		s.computePsuedoNormals();
		s.redistance();
	} else {
		buildTree(s.tree, BfastVector3(atof(argv[2])), BfastVector3(atof(argv[3])), atoi(argv[4]), phi);
		s.contourTree(phi);
		s.redistance();
	}

	ofstream out(argv[1], ios::out | ios::binary);
	s.write(out);
	out.close();

	return 0;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: setup.cpp,v $
// Revision 1.4  2006/06/16 21:07:13  adamb
// new version
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
// Revision 1.1.1.1  2005/05/25 05:41:01  adamb
// Initial Revision
//
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
