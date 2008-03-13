#include "debug.h"

Trace_Space debug_space;

#define MAX_DKEY 256
int dkey[MAX_DKEY];

void dkey_toggle(int c)
{
    if(0 < c && c <MAX_DKEY)
	{
	    dkey[c]^=1;
	    DEBUG(cerr << LOCALISATION << "(" << (char)c << ") ->" <<  dkey[c] <<endl);
	}
}

ostream & operator <<(ostream &out, const Trace_Space &s)
{
      unsigned int c=0;
      for(c=0;c<4*s.count;c++)
	    out << ' ';
      return out;
}
