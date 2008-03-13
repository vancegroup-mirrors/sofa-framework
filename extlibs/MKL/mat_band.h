#ifndef __MATRIX_BAND_H__
#define __MATRIX_BAND_H__

#include "mat_gen.h"

// Find L U = A (A band matrix of large k)
// L is lower diagonal (band matrix of large k/2 + 1)
// U is upper diagonal (band matrix of large k/2 + 1) 
template <class Matrix1, class Matrix2, class Matrix3, class Vector_int>
inline void matrix_band_find_LU(const int n, const int k, const Matrix1 &a, Matrix2 &L, Matrix3 &U, Vector_int &t);

// Resolve U X = B,
// U is upper diagonal (band of large k/2 + 1)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_band_resolve_upper(const int n,const int k,const Matrix &U, const Vector1 &b, Vector2 &x);    

// Resolve L X = B
// L is lower diagonal (band of large k/2 + 1)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_band_resolve_lower(const int n,const int k,const Matrix &L, const Vector1 &b, Vector2 &x);

// Resolve L U X = B
// U is upper diagonal (band of large k/2 + 1)
// L is lower diagonal (band of large k/2 + 1)
// temp must be a vector of dimension n
template <class T,class Matrix1, class Matrix2, class Vector_int, class Vector1, class Vector2, class Vector3>
inline void matrix_band_resolve_LU_T(const T*p,
				     const int n,const int k,
				     const Matrix1 &L,
				     const Matrix2 &U,
				     const Vector_int &t,
				     const Vector1 &b,
				     Vector2 &x,
				     Vector3 &temp);

#define macro_matrix_band_resolve_LU(T, n, k, L, U, t, b, x, temp) matrix_band_resolve_LU_T( (const T*)0, n, k, L, U, t, b, x, temp)

#include "mat_band.hpp"

#endif
