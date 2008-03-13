#ifndef _VECT_MAT_MKL_H_
#define _VECT_MAT_MKL_H_

#define MAT_TRANSPOSE 1
#define MAT_NO_TRANSPOSE 0

#include "vect_dyn.h"
#include "mat_dyn.h"

/* fonction : b<- a * scaleFactor + b */
template <class T>
void vector_add_scaleMkl( int length, double scaleFactor, Dynamic_Vector<T> &a, const Dynamic_Vector<T> &b);

template <class T>
void matrix_mulMkl(const unsigned int n_rows, const unsigned int n_columns, const unsigned  int other_n_columns, const Dynamic_Matrix<T> &a, const Dynamic_Matrix<T> &b, Dynamic_Matrix<T> &c);

template <class T>
void matrix_mul_vectorMkl(const unsigned int n_rows, const unsigned int n_columns, const Dynamic_Matrix<T> &a, const Dynamic_Vector<T> &b, Dynamic_Vector<T> &c, int paramTransA);

template <class T>
void vector_copyMkl(int length, const Dynamic_Vector<T> &src, Dynamic_Vector<T> &dst);

template <class T>
void vector_dot_productMkl (const unsigned int length, const Dynamic_Vector<T> &a, const Dynamic_Vector<T> &b, double &res);

template <class T>
void vector_normMkl (const unsigned int length, const Dynamic_Vector<T> &a, double &norm);

#include "vect_mat_mkl.hpp"

#endif /* #ifndef _VECT_MAT_MKL_H_ */
