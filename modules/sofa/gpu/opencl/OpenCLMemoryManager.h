#ifndef SOFA_GPU_OPENCLMEMORYMANAGER_H
#define SOFA_GPU_OPENCLMEMORYMANAGER_H

#include "myopencl.h"
#include <sofa/helper/MemoryManager.h>
#include <iostream>
#include <stdlib.h>
#include "OpenCLCommon.h"
	
namespace sofa {

namespace gpu {
  
namespace opencl {

#define DEBUG_TEXT(t) //printf("\t%s\t %s %d\n",t,__FILE__,__LINE__);

void OpenCLMemoryManager_memsetDevice(int d, _device_pointer a, int value, size_t n);

/**
  * OpenCLMemoryManager 
  * OpenCL and multi-GPU version of MemoryManager
  */
template <class T>
class OpenCLMemoryManager: public sofa::helper::MemoryManager<T>
{
public:

    template<class T2> struct rebind {
	typedef OpenCLMemoryManager<T2> other;
    };

	typedef T* host_pointer;

	typedef _device_pointer device_pointer ;

	typedef GLuint gl_buffer;

	enum { MAX_DEVICES = 8 };
	enum { BSIZE = intBSIZE };
	enum { SUPPORT_GL_BUFFER = 0 };

	static int numDevices()
	{
//		DEBUG_TEXT("OpenCLMemoryManager::numDevice");
		return gpu::opencl::myopenclNumDevices();
	}
	
	static void hostAlloc(void ** hPointer,int n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::hostAlloc");
		*hPointer = (host_pointer) malloc(n);
		//std::cout << "[" <<*hPointer << "]" << std::endl;
	}
	
	static void memsetHost(host_pointer hPointer, int value,size_t n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::memsetHost");
		memset((void*) hPointer, value, n);
		//std::cout << "[" <<hPointer << "]" << std::endl;
	}
	
	static void hostFree(const host_pointer hSrcPointer)
	{
		DEBUG_TEXT("OpenCLMemoryManager::hostFree");
		//std::cout << "[" <<hSrcPointer << "]" << std::endl;
		free(hSrcPointer);
	}
	

	static void deviceAlloc(int d,device_pointer* dPointer, int n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::deviceAlloc");
		myopenclCreateBuffer(d,&(*dPointer).m,n);
		dPointer->offset=0;
		dPointer->_null=false;
	}
	
	static void deviceFree(int d,/*const*/ device_pointer dSrcPointer)
	{
		DEBUG_TEXT("OpenCLMemoryManager::deviceFree");
		myopenclReleaseBuffer(d,dSrcPointer.m);
		dSrcPointer.offset=0;
		dSrcPointer._null=true;
	}
	
	static void memcpyHostToDevice(int d, device_pointer dDestPointer,const host_pointer hSrcPointer, size_t n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::memcpyHostToDevice");
		myopenclEnqueueWriteBuffer(d,(dDestPointer).m,(dDestPointer).offset,hSrcPointer,n);
	}
	
	static void memcpyDeviceToHost(int d, host_pointer hDestPointer, const device_pointer dSrcPointer, size_t n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::memcpyDeviceToHost");
		myopenclEnqueueReadBuffer(d,hDestPointer,(dSrcPointer).m,(dSrcPointer).offset,n);
	}
	
	static void memcpyDeviceToDevice(int d, device_pointer dDestPointer, const device_pointer dSrcPointer , size_t n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::memcpyDeviceToDevice");
		myopenclEnqueueCopyBuffer(d, (dDestPointer).m,(dDestPointer).offset, (dSrcPointer).m,(dSrcPointer).offset, n);
	}
	
	static void memsetDevice(int d, device_pointer dDestPointer, int value, size_t n)
	{
		DEBUG_TEXT("OpenCLMemoryManager::memsetDevice");
		OpenCLMemoryManager_memsetDevice(d, dDestPointer, value, n);
	}

//	static void memsetDevice(int d, device_pointer dDestPointer, int value, size_t n)
//	{
//		myopenclMemsetDevice(d,dDestPointer,value,n);
//	}



/*	static void memsetDevice(int d, device_pointer dDestPointer, int value, size_t n)
	{
		T* array = new T[n];
		memset((void*)array, value, n);
		myopenclEnqueueWriteBuffer(d,(dDestPointer).m,(dDestPointer).offset,(void*)array,n);
		delete(array);
	}*/


	
	static int getBufferDevice() {
		DEBUG_TEXT("OpenCLMemoryManager::getBifferDevice");
	    return 0;
	}
	
	static bool bufferAlloc(gl_buffer*/* bId*/, int/* n*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferAlloc");
	    return false;
	}
	
	static void bufferFree(const gl_buffer /*bId*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferFree");
	}
	
	static bool bufferRegister(const gl_buffer /*bId*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferRegister");
	    return false; 
	}
	
	static void bufferUnregister(const gl_buffer /*bId*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferUnregister");
	}
	
        static bool bufferMapToDevice(device_pointer* dDestPointer, const gl_buffer /*bSrcId*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferMapToDevice");
		device_pointer* d=(device_pointer*)dDestPointer;
		d->m=d->m;		//delete this line when implementation
	    return false; 
	}
	
        static void bufferUnmapToDevice(device_pointer*  dDestPointer, const gl_buffer /*bSrcId*/) {
		DEBUG_TEXT("OpenCLMemoryManager::bufferUnmapToDevice");
		device_pointer* d=(device_pointer*)dDestPointer;
		d->m=d->m;		//delete this line when implementation
	}

	static device_pointer deviceOffset(device_pointer memory,size_t offset)
	{
		device_pointer p;
		p.m = memory.m;
		p.offset = memory.offset + offset*sizeof(T);
		return p;
	}

	static device_pointer null()
	{
		return device_pointer();
	}

	static bool isNull(device_pointer p)
	{
		return p._null;
	}

};

}

}

}

#endif
#undef DEBUG_TEXT
