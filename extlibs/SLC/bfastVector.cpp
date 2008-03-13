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

#include "bfastVector.h"

namespace SLC {

using namespace std;

//-------------------------------------------------------------------


inline static
istream &eatChar(char c,istream &buf) {
  char r;
  buf >> r;
  if (r!=c) {
    buf.clear(buf.rdstate() | ios::failbit);
  }
  return buf;
}

//-------------------------------------------------------------------

istream &operator>>(istream &strm,BfastVector3 &v) {
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
  

ostream &operator<<(ostream &strm,const BfastVector3 &v) {
  strm << "[";
  strm << v[0]; strm << ",";
  strm << v[1]; strm << ",";
  strm << v[2]; strm << "]";
  return strm;
}

//-------------------------------------------------------------------


istream &operator>>(istream &strm,BfastVector2 &v) {
  ios::fmtflags orgFlags = strm.setf(ios::skipws);
  eatChar('[',strm);
  strm >> v[0];
  eatChar(',',strm);
  strm >> v[1];
  eatChar(']',strm);
  strm.flags(orgFlags);
  return strm;
}
  

ostream &operator<<(ostream &strm,const BfastVector2 &v) {
  strm << "[";
  strm << v[0]; strm << ",";
  strm << v[1]; strm << "]";
  return strm;
}

//-------------------------------------------------------------------

}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// RCS Revision History
//
// $Log: bfastVector.cpp,v $
// Revision 1.1.1.1  2005/09/06 22:45:12  adamb
// imported sources
//
// Revision 1.1  2005/09/01 21:06:46  adamb
// SmVector->BfastVector
//
// Revision 1.1.1.1  2005/05/25 05:41:04  adamb
// Initial Revision
//
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
