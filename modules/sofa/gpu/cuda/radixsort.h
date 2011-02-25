/*
* Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
*
* NOTICE TO USER:   
*
* This source code is subject to NVIDIA ownership rights under U.S. and 
* international Copyright laws.  
*
* NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
* CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
* IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
* REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
* MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
* IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
* OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
* OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
* OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
* OR PERFORMANCE OF THIS SOURCE CODE.  
*
* U.S. Government End Users.  This source code is a "commercial item" as 
* that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
* "commercial computer software" and "commercial computer software 
* documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
* and is provided to the U.S. Government only as a commercial end item.  
* Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
* 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
* source code with only those rights set forth herein.
*/
#ifndef   __RADIXSORT_H__
#define   __RADIXSORT_H__

// -----------------------------------------------------------------------
// Fast CUDA Radix Sort Class
//
// The parallel radix sort algorithm implemented by this code is described
// in the following paper.
//
// Satish, N., Harris, M., and Garland, M. "Designing Efficient Sorting 
// Algorithms for Manycore GPUs". In Proceedings of IEEE International
// Parallel & Distributed Processing Symposium 2009 (IPDPS 2009).
//
// -----------------------------------------------------------------------

extern "C"
int radixSortTempStorage(unsigned int numElements);

extern "C"
void radixSort(unsigned int *keys, 
               unsigned int *values, 
               unsigned int *temp, 
               unsigned int numElements, 
               unsigned int keyBits = 32,
               bool         flipBits = false);

extern "C"
void radixSortFloatKeys(float        *keys, 
                        unsigned int *values, 
                        unsigned int *temp, 
                        unsigned int numElements, 
                        unsigned int keyBits = 32,
                        bool         negativeKeys = true);

extern "C"
void radixSortKeysOnly(unsigned int *keys, 
                       unsigned int *temp, 
                       unsigned int numElements, 
                       unsigned int keyBits = 32,
                       bool         flipBits = false);

extern "C"
void radixSortFloatKeysOnly(float        *keys, 
                            unsigned int *temp, 
                            unsigned int numElements, 
                            unsigned int keyBits = 32,
                            bool         negativeKeys = true);

#endif // __RADIXSORT_H__
