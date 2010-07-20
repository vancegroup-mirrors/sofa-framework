// VRPN Imager Server example program.

#include <stdio.h>
#include "vrpn_Connection.h"
#include "vrpn_Imager.h"
#include <stdlib.h>
#include <stdint.h>

static	vrpn_Connection	    *g_connection = NULL;
static	vrpn_Imager_Server  *g_is = NULL;
const	unsigned	    g_sizeX = 1024 , g_sizeY = 768;
const   unsigned	    g_strip = 1;

// Fill a buffer of 8-bit integers with an image that has a diagonal
// line of growing brightness.  The size of the image is passed in, as
// well as an offset for the brightnesses to make the image change
// a bit over time.

static	void  fill_buffer(vrpn_uint8 *buffer,
			  const unsigned numX, const unsigned numY,
			  const unsigned offset)
{
  unsigned x,y;
  for (x = 0; x < numX; x++) {
    for (y = 0; y < numY; y++) {
      buffer[x + y*numX] = (x + y + offset) % 256;
    }
  }
  
}

#ifndef min
#define	min(a,b) ( (a) < (b) ? (a) : (b) )
#endif
int main (int argc, char * argv[])
{
  static  int frame_number = 0;
  static  vrpn_uint8 bufferRed[g_sizeX*(g_sizeY+g_strip)];
  static  vrpn_uint8 bufferGreen[g_sizeX*(g_sizeY+g_strip)];
  static  vrpn_uint8 bufferBlue[g_sizeX*(g_sizeY+g_strip)];
  int	  channel_id[3];


  // Need to have a global pointer to it so we can shut it down
  // in the signal handler (so we can close any open logfiles.)
  //vrpn_Synchronized_Connection	connection;
  if ( (g_connection = vrpn_create_server_connection()) == NULL) {
    fprintf(stderr, "Could not open connection\n");
    return -1;
  }

  // Open the imager server and set up channel zero to send our data.
  if ( (g_is = new vrpn_Imager_Server("TestImage", g_connection, g_sizeX, g_sizeY+g_strip)) == NULL) {
    fprintf(stderr, "Could not open imager server\n");
    return -1;
  }
  if ( (channel_id[0] = g_is->add_channel("red")) == -1) {
    fprintf(stderr, "Could not add channel\n");
    return -1;
  }
  if ( (channel_id[1] = g_is->add_channel("green")) == -1) {
    fprintf(stderr, "Could not add channel\n");
    return -1;
  }
  if ( (channel_id[2] = g_is->add_channel("blue")) == -1) {
    fprintf(stderr, "Could not add channel\n");
    return -1;
  }



  // Generate about ten frames per second by sending one and then sleeping.
  // Better would be to mainloop() the connection much faster, and then
  // only send when it is time.  This will do, but the connections might
  // not open as fast as they would otherwise.

  // It would be nice to let the user stop the server cleanly, but ^C
  // will do the trick.

    fprintf(stderr, "sizeof float = %ld\n", sizeof(float));
    fprintf(stderr, "sizeof vrpn_uint8 = %ld\n", sizeof(vrpn_uint8 ));
    fprintf(stderr, "sizeof unsigned char = %ld\n", sizeof(unsigned char));
    fprintf(stderr, "sizeof uint32_t= %ld\n", sizeof(uint32_t));

    uint32_t numData = 7;
    float *data  = (float *) calloc(numData, sizeof(float));

    if (numData * sizeof(float) > g_sizeY) {
	fprintf(stderr, "Too many data to send in one image line!!!");
    }

    for (unsigned int i = 0; i < numData; i++) {
	    data[i] = 0;
            fprintf(stderr,"[%d] = %f\n", i, data[i]);
    }

    unsigned char myOffset = sizeof(uint32_t);
    memcpy(bufferRed+g_sizeX*g_sizeY, &numData, myOffset);
    //for (unsigned i = 0; i < 28; i++)
    //	fprintf(stderr, "CH [%d] = %i\n", i, bufferRed[g_sizeX*g_sizeY+i]);
    


  
  while (1) {
    fill_buffer(bufferRed, g_sizeX, g_sizeY, frame_number++ % 256);
    data[0] += 0.1;
    memcpy(bufferRed+g_sizeX*g_sizeY+myOffset, data, numData*sizeof(float));
    //fill_buffer(bufferGreen, g_sizeX, g_sizeY, frame_number++ % 256);
    //fill_buffer(bufferBlue, g_sizeX, g_sizeY, frame_number++ % 256);




    /*for (unsigned i=0; i < g_strip*g_sizeX; i++) {
	bufferRed[g_sizeX*g_sizeY+i] = 0;
	bufferGreen[g_sizeX*g_sizeY+i] = 255;
	bufferBlue[g_sizeX*g_sizeY+i] = 0;
    }*/

    //send each channel separately
    g_is->send_begin_frame(0, g_sizeX-1, 0, g_sizeY-1+g_strip);
    g_is->mainloop();
    int nRowsPerRegion=vrpn_IMAGER_MAX_REGIONu8/g_sizeX;
    int y;
    for(y=0; y<g_sizeY+g_strip; y+=nRowsPerRegion) {
      g_is->send_region_using_base_pointer(channel_id[0],0,g_sizeX-1,y,min(g_sizeY+g_strip,y+nRowsPerRegion)-1, bufferRed, 1, g_sizeX, g_sizeY+g_strip);
      g_is->send_region_using_base_pointer(channel_id[1],0,g_sizeX-1,y,min(g_sizeY+g_strip,y+nRowsPerRegion)-1, bufferGreen, 1, g_sizeX, g_sizeY+g_strip);
      g_is->send_region_using_base_pointer(channel_id[2],0,g_sizeX-1,y,min(g_sizeY+g_strip,y+nRowsPerRegion)-1, bufferBlue, 1, g_sizeX, g_sizeY+g_strip);
      g_is->mainloop();
    }
    g_is->send_end_frame(0, g_sizeX-1, 0, g_sizeY-1+g_strip);

    g_is->mainloop();
    g_connection->mainloop();
    vrpn_SleepMsecs(10);
  }

  return 0;
}
