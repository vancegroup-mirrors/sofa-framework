#ifndef __DEBUG_H_
#define __DEBUG_H_

#include "portable_iostream.h"

struct Trace_Space
{
      unsigned int count;
      Trace_Space() :count(0) {}
};

ostream & operator <<(ostream &out, const Trace_Space &s);

extern int dkey[];
extern Trace_Space debug_space;

void dkey_toggle(int c);

#ifndef NODEBUG

#ifdef __GNUC__
#define LOCALISATION __FILE__ << ':' << __LINE__ << ": in " << __PRETTY_FUNCTION__
#elif defined(WIN32) || defined(SGI_CC)
# define LOCALISATION __FILE__ << ':' << __LINE__
#else
# define LOCALISATION __FILE__ << ':' << __LINE__  << ": in " << __FUNCTION__
#endif

#define EXIT(msg) { cerr << LOCALISATION << ", "<< msg << endl; exit(2); } 
#define WARNING(msg) { cerr << LOCALISATION << ", "<< msg <<endl; }
#define CHECK(bexp,msg) { if(!(bexp)) { WARNING(msg);} }
#define REQUIRE(bexp) { if(!(bexp)) WARNING( "REQUIRE BROKEN" ); }
#define ENSURE(bexp) { if(!(bexp)) WARNING( "ENSURE BROKEN" ); }

#define TRACE_begin if( TRACE ){ cerr << debug_space << "begin ("<< debug_space.count << ") " <<LOCALISATION << endl; debug_space.count++;}
#define TRACE_end if( TRACE ) { debug_space.count--; cerr << debug_space << "end ("<< debug_space.count << ") "<<  LOCALISATION << endl; }

#define DEBUG(x) {cerr << LOCALISATION << flush; x;}
#define DEBUG_ON_KEY(key,x) { if(dkey[key]) { cerr << LOCALISATION << flush; x;} }

#else

#define LOCALISATION ""

#define EXIT(msg) { exit(2); }
#define WARNING(msg)
#define CHECK(bexp,msg)
#define REQUIRE(bexp)
#define ENSURE(bexp)

#define TRACE_begin
#define TRACE_end

#define DEBUG(x)
#define DEBUG_ON_KEY(key,x)

#endif



#endif
