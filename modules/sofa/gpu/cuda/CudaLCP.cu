#include "CudaCommon.h"
#include "CudaMath.h"
#include <stdio.h>

#if defined(__cplusplus)
namespace sofa
{
namespace gpu
{
namespace cuda
{
#endif

extern "C"
{
	void CudaLCP_MultVector(int dim,int index, const void * m,const void * f,void * r);
	int CudaLCP_MultVector_ResSize(unsigned int dim);
	void CudaLCP_ComputeError(int compteur2,int sizeTmp, const void * tmp, const void * M,const void * q,void * f,void * res,void * error);
}

__global__ void CudaLCP_MultVector_kernel(int dim, int i, const float * m,const float * f, float * r, int offset) {
	//! Dynamically allocated shared memory for gather
	extern  __shared__  float temp[];
	int index0 = umul24(blockIdx.x,blockDim.x);
	int index1 = threadIdx.x;
	int n = blockDim.x; //min(blockDim.x , size-index0);
	float acc = 0;
	int index = index0+index1;
	if (index < dim && index != i)
		acc = m[index] * f[index];

	while(offset>0)
	{
		if (index1 >= offset && index1 < n)
			temp[index1] = acc;
		__syncthreads();
		if (index1+offset < n)
			acc += temp[index1+offset];
		n = offset;
		offset >>= 1;
	}
	if (index1 == 0)
		r[blockIdx.x] = acc;
}

__global__ void CudaLCP_ComputeError_kernel(int compteur2, const float * tmp,const float * M, const float * q,float * f, float * res, float * error, int offset) {
	//! Dynamically allocated shared memory for gather
	extern  __shared__  float temp[];
	int index1 = threadIdx.x;
	int n = blockDim.x;
	float r;
	
	r = tmp[index1];
	
	while(offset>0)
	{
		if (index1 >= offset && index1 < n)
			temp[index1] = r;
		__syncthreads();
		if (index1+offset < n)
			r += temp[index1+offset];
		n = offset;
		offset >>= 1;
	}
	
	if (index1==0) {
		r += q[compteur2];
		//for (int i=0;i<sizeTmp;++i)	r += tmp[i];			
		res[compteur2] = r;
		
		float f_1 = f[compteur2];

		float f_new;
		float MindM = M[0];
		
		if (r<0) f_new=-r/MindM;
		else f_new=0.0;
			
		f[compteur2] = f_new;
		
		error[0] += fabs(MindM * (f_new - f_1) );		
	}
}

int CudaLCP_MultVector_ResSize(unsigned int dim) {
	return (dim+BSIZE-1)/BSIZE;
}

void CudaLCP_MultVector(int dim,int index, const void * m,const void * f,void * r) {
	dim3 threads(BSIZE,1);
	dim3 grid((dim+BSIZE-1)/BSIZE,1);

	CudaLCP_MultVector_kernel<<< grid, threads, threads.x*sizeof(float) >>>(dim, index, (const float*)m, (const float*)f, (float*)r, BSIZE/2);
}

void CudaLCP_ComputeError(int compteur2,int sizeTmp, const void * tmp, const void * M,const void * q,void * f,void * res,void * error) {
	dim3 threads(sizeTmp,1);
	dim3 grid(1,1);
	int offset;
	if (sizeTmp==1)
		offset = 0;
	else
	{
		offset = 1;
		while (offset*2 < sizeTmp)
			offset *= 2;
	}

	CudaLCP_ComputeError_kernel<<< grid, threads, threads.x*sizeof(float) >>>(compteur2,(const float*)tmp, (const float*)M,(const float*)q, (float*)f, (float*)res,(float*)error,offset);
}

#if defined(__cplusplus)
} // namespace cuda
} // namespace gpu
} // namespace sofa
#endif
