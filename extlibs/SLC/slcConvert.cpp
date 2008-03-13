//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// slcConvert
//   -- Code File
//  
// Primary Author: Adam Bargteil (adamb@cs.berkeley.edu)
// 
// This is a little utility to convert between several file formats
// supported by the surface tracking code.  It looks at the
// file extensions to determine the file types.
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


#include <fstream>
#include <iostream>
#include "slcSurface.h"

using namespace sofa::helper::slc;

int main(int argc, char *argv[]) {
#ifdef INVENTOR
	SoDB::init();
#endif
	if (argc != 3) {
		std::cerr<<"Usage: "<<argv[0]<<" [inputfile] [outputfile]"<<std::endl;
		exit(1);
	}
	bool COLORS=false;
	DtTree t;
	SlcSurface s(&t);
	BfastVecList colors;
	if (strstr(argv[1], "bvt"))
		s.bvtRead(argv[1]);
	else if (strstr(argv[1], "obj"))
		readObjFile(argv[1], s.meshPts, s.triangles);
	else if (strstr(argv[1], "poly"))
		gtRead(argv[1], s.meshPts, s.triangles);
	else if (strstr(argv[1], "bdt")) {
		std::ifstream fin(argv[1], std::ios::in| std::ios::binary);
		s.read(fin);
		fin.close();
	}	else if (strstr(argv[1], "col")) {
		std::ifstream fin(argv[1], std::ios::in| std::ios::binary);
		colors.read(fin);
		COLORS = true;
		fin.close();
	}
	s.computeNormals();
	if (strstr(argv[2], "bvt"))
		s.bvtDump(argv[2]);
	else if (strstr(argv[2], "rib")) 
		if (!COLORS)
			s.ribDump(argv[2]);
		else {
			std::ofstream fout(argv[2], std::ios::out);
			fout<<"\"Cs\" [";
			for (int i=0; i<colors.numVecs(); i++)
				fout<<colors[i][0]<<" "<<colors[i][1]<<" "<<colors[i][2]<<" ";
			fout<<"]"<<std::endl;
		}
	else if (strstr(argv[2], "bdt")) {
		std::ofstream fout(argv[2], std::ios::out | std::ios::binary);
		s.write(fout);
		fout.close();
	} else if (strstr(argv[2], "obj")) 
		s.objDump(argv[2]);
	else if (strstr(argv[2], "bpoly")) 
		bgtDump(argv[2], s.meshPts, s.triangles);
	else if (strstr(argv[2], "poly")) 
		gtDump(argv[2], s.meshPts, s.triangles);
	else if (strstr(argv[2], "iv")) {
#ifdef INVENTOR
		SoSeparator *iv = s.ivTree();
		iv->ref();
		bfastWriteIVFile(argv[2], iv);
		iv->unref();
#else
		std::cerr<<"inventor files not supported, sorry."<<std::endl;
#endif
	}
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: slcConvert.cpp,v $
// Revision 1.2  2006/06/16 21:07:13  adamb
// new version
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
