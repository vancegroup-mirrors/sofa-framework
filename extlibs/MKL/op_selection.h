#ifndef __OP_SELECTION_H_
#define __OP_SELECTION_H_

#include "mem16.h"

#ifdef SSE2
	#include "op_ia32.h"
	#define _SSE_(x) x
	#define _SSE2_(x) x
	#define _GENERIC_SSE_(x)
	#define _GENERIC_SSE2_(x)
#elsif defined(SSE)
	#include "op_ia32.h"
	#define _SSE_(x) x
	#define _SSE2_(x)
	#define _GENERIC_SSE_(x)
	#define _GENERIC_SSE2_(x) x
#else
	#define _SSE_(x)
	#define _SSE2_(x)
	#define _GENERIC_SSE_(x) x
	#define _GENERIC_SSE2_(x) x
#endif

#include "op_generic.h"

//Optimisation float

inline float vector_norm_square(unsigned int size, float *a) {
	_SSE_(return sse_vector_norm_square(size,a));
	_GENERIC_SSE_(return generic_vector_norm_square(size,a));
}

inline void vector_copy(const unsigned int size, float *a, float * b) {
	_SSE_(sse_vector_copy(size,a,b));
	//_SSE_(generic_vector_copy(size,a,b));
	_GENERIC_SSE_(generic_vector_copy(size,a,b));
}

inline void vector_neg(const unsigned int size, float *a, float * b) {
	_SSE_(sse_vector_neg(size,a,b));
	//_SSE_(generic_vector_neg(size,a,b));
	_GENERIC_SSE_(generic_vector_neg(size,a,b));
}

inline void vector_add(const unsigned int size, float *a, float * b, float * c) {
	_SSE_(sse_vector_add(size,a,b,c));
	//_SSE_(generic_vector_add(size,a,b,c));
	_GENERIC_SSE_(generic_vector_add(size,a,b,c));
}	

inline void vector_sub(const unsigned int size, float *a, float * b, float * c) {
	_SSE_(sse_vector_sub(size,a,b,c));
	//_SSE_(generic_vector_sub(size,a,b,c));
	_GENERIC_SSE_(generic_vector_sub(size,a,b,c));
}

inline void vector_mul(const unsigned int size, float *a, float * b, float * c) {
	_SSE_(sse_vector_mul(size,a,b,c));
	//_SSE_(generic_vector_mul(size,a,b,c));
	_GENERIC_SSE_(generic_vector_mul(size,a,b,c));
}

inline void vector_mul_k(const unsigned int size, float *a, float k, float *c) {
	_SSE_(sse_vector_mul_k(size,a,k,c));
	//_SSE_(generic_vector_mul_k(size,a,k,c));
	_GENERIC_SSE_(generic_vector_mul_k(size,a,k,c));
}

inline float vector_scalar_product_fast(unsigned int size, float *a, float *b) {
	//_SSE_(return  sse_vector_scalar_product(size, a, b));
	_SSE_(return  generic_vector_scalar_product1(size, a, b));
	_GENERIC_SSE_(return  generic_vector_scalar_product1(size, a, b));
}

inline void vector_add_mul_k(unsigned int size, float *a, float k, float *b, float *c) {
	//_SSE_(sse_vector_add_mul_k(size,a,k,b,c));
	_SSE_(generic_vector_add_mul_k(size,a,k,b,c));
	_GENERIC_SSE_(generic_vector_add_mul_k(size,a,k,b,c));
}

// Optimisations double

inline double vector_norm_square(unsigned int size, double *a) {
	//_SSE2_(return sse_vector_norm_square(size,a));
	//_SSE2_(return generic_vector_norm_square(size,a));
	_GENERIC_SSE2_(return generic_vector_norm_square(size,a));
}

inline void vector_copy(const unsigned int size, double *a, double * b) {
	//_SSE2_(sse_vector_copy(size,a,b));
	//_SSE2_(generic_vector_copy(size,a,b));
	_GENERIC_SSE2_(generic_vector_copy(size,a,b));
}

inline void vector_neg(const unsigned int size, double *a, double * b) {
	//_SSE2_(sse_vector_neg(size,a,b));
	//_SSE2_(generic_vector_neg(size,a,b));
	_GENERIC_SSE2_(generic_vector_neg(size,a,b));
}

inline void vector_add(const unsigned int size, double *a, double * b, double * c) {
	//_SSE2_(sse_vector_add(size,a,b,c));
	//_SSE2_(generic_vector_add(size,a,b,c));
	_GENERIC_SSE2_(generic_vector_add(size,a,b,c));
}	

inline void vector_sub(const unsigned int size, double *a, double * b, double * c) {
	//_SSE2_(sse_vector_sub(size,a,b,c));
	//_SSE2_(generic_vector_sub(size,a,b,c));
	_GENERIC_SSE2_(generic_vector_sub(size,a,b,c));
}

inline void vector_mul(const unsigned int size, double *a, double * b, double * c) {
	//_SSE2_(sse_vector_mul(size,a,b,c));
	//_SSE2_(generic_vector_mul(size,a,b,c));
	_GENERIC_SSE2_(generic_vector_mul(size,a,b,c));
}

inline void vector_mul_k(const unsigned int size, double *a, double k, double *c) {
	//_SSE2_(sse_vector_mul_k(size,a,k,c));
	//_SSE2_(generic_vector_mul_k(size,a,k,c));
	_GENERIC_SSE2_(generic_vector_mul_k(size,a,k,c));
}

inline double vector_scalar_product_fast(unsigned int size, double *a, double *b) {
	//_SSE2_(return  sse_vector_scalar_product(size, a, b));
	//_SSE2_(return  generic_vector_scalar_product1(size, a, b));
	_GENERIC_SSE2_(return generic_vector_scalar_product1(size, a, b));
}


inline void vector_add_mul_k(unsigned int size, double *a, double k, double *b, double *c) {
	//_SSE2_(sse_vector_add_mul_k(size,a,k,b,c));
	//_SSE2_(generic_vector_add_mul_k(size,a,k,b,c));
	_GENERIC_SSE2_(generic_vector_add_mul_k(size,a,k,b,c));
}

#endif
