//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Surface
//   -- Code File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
// 
// This is a sample driver program for the surface tracking library.
// It uses the velocity field of the "Enright" test to advect the surface.
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

using namespace std;
using namespace sofa::helper::slc;

SlcSurface *newSurface, *oldSurface;
DtTree *newTree, *oldTree;
BfastReal dt;
int ITER;

BfastVector3 vel(const BfastVector3 &x, void *data) {
	BfastVector3 ret;

	// "Enright" test
	ret[0] = 2*sin(M_PI*x[0])*sin(M_PI*x[0])*sin(2*M_PI*x[1])*sin(2*M_PI*x[2]);
	ret[1] = -sin(2*M_PI*x[0])*sin(M_PI*x[1])*sin(M_PI*x[1])*sin(2*M_PI*x[2]);
	ret[2] = -sin(2*M_PI*x[0])*sin(2*M_PI*x[1])*sin(M_PI*x[2])*sin(M_PI*x[2]);
	if (ITER > 100) ret *= -1.0;
	return ret;
}

BfastReal phi(const BfastVector3 &x, void *data) {
	BfastVector3 y = bfastTraceBack(x,vel,dt);
	return oldSurface->eval(y);
}

int main(int argc, char *argv[]) {
	if (argc != 4) {
		cerr<<"Usage: "<<argv[0]<<" [inputfile] [timestep] [iterations]"<<endl;
		exit(1);
	}
	ifstream in(argv[1],ios::in | ios::binary);
	dt = atof(argv[2]);
	int iter = atoi(argv[3]);
	char fname[80];
	int frame=1;
	BfastReal lc, uc;
	oldTree = new DtTree();
	oldSurface = new SlcSurface(oldTree);
	oldSurface->read(in);
	in.close();

	for (ITER=1; ITER<=iter; ITER++) {
		cout<<"step "<<ITER<<endl;
		newTree = new DtTree();
		newSurface = new SlcSurface(newTree);
		timeval startTime, endTime;

		gettimeofday(&startTime, NULL);

		buildTree(newSurface->tree, oldSurface->tree->lc, oldSurface->tree->uc, 
							oldSurface->tree->max_level, phi);
		gettimeofday(&endTime, NULL);
		cout<<"build tree took "<<(endTime.tv_sec-startTime.tv_sec)+
			(endTime.tv_usec-startTime.tv_usec)*1.0e-6<<endl;

		gettimeofday(&startTime, NULL);
		newSurface->contourTree(phi);
		gettimeofday(&endTime, NULL);
		cout<<"contour tree took "<<(endTime.tv_sec-startTime.tv_sec)+
			(endTime.tv_usec-startTime.tv_usec)*1.0e-6<<endl;

		delete oldSurface;
		delete oldTree;
		oldSurface = newSurface;
		oldTree = newTree;

		sprintf(fname, "foo.1%04d.bvt", frame);
		newSurface->bvtDump(fname);

		gettimeofday(&startTime, NULL);
		newSurface->redistance();
		gettimeofday(&endTime, NULL);
		cout<<"redistance tree took "<<(endTime.tv_sec-startTime.tv_sec)+
			(endTime.tv_usec-startTime.tv_usec)*1.0e-6<<endl;

		frame++;
	}
	cout<<"program exiting"<<endl;
	return 0;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: surface.cpp,v $
// Revision 1.3  2006/06/16 21:07:14  adamb
// new version
//
// Revision 1.2  2005/12/29 01:59:38  adamb
// Added void * to all passed in functions
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
// Revision 1.1.1.1  2005/05/25 05:41:05  adamb
// Initial Revision
//
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
