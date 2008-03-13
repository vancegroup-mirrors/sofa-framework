#ifndef __MATRIX_H_
#define __MATRIX_H_

#include <iostream>
//#include "portable_iostream.h"
//#include "portable_bool.h"

template <class Matrix>
ostream & matrix_put(ostream & out, const unsigned int n_rows, const unsigned int n_columns, Matrix &m);
    
template <class Matrix>
inline void matrix_null(const unsigned int n_rows, const unsigned int n_columns, Matrix &a);

template <class Matrix>
inline void matrix_null(Matrix &a);

template <class Matrix>
inline void matrix_identity(const unsigned int n_rows, const unsigned int n_columns, Matrix &a);

template <class Matrix>
inline void matrix_identity(Matrix &a);

template < class T, class Matrix>
inline void matrix_rotation_x(Matrix &m, T& angle);

template < class T, class Matrix>
inline void matrix_rotation_y(Matrix &m, T& angle);

template < class T, class Matrix>
inline void matrix_rotation_z(Matrix &m, T& angle);

template <class Matrix1, class Matrix2>
inline bool matrix_is_equal(const unsigned int n, const unsigned int m, const Matrix1 &a, const Matrix2 &b);
    
template <class Matrix, class T>
inline void matrix_fill(const unsigned int n_rows, const unsigned int n_columns, const T &k, Matrix &a);
    
template <class Matrix1, class Matrix2>
inline void matrix_copy(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix2 &b);

template <class Matrix1, class Matrix2>
inline void matrix_copy_to_array(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix2 b);
    
template <class Matrix1, class Matrix3>
inline void matrix_transpose(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix3 &c);  

template <class Matrix1, class Matrix3>
inline void matrix_transpose(const Matrix1 &a, Matrix3 &c);  

template <class Matrix1, class Matrix2>
inline void matrix_set(const unsigned int n_rows, const unsigned int n_columns,
		       const Matrix1& sub_matrix,
		       const unsigned int i, const unsigned int j,
		       Matrix2 &m);

template <class Matrix1, class Matrix2>
inline void matrix_set(const Matrix1& sub_matrix,
		       const unsigned int i, const unsigned int j,
		       Matrix2 &m);


template <class Matrix, class Vector>
inline void matrix_set_vector_column(const unsigned int length,
				     const Vector &vect,
				     const unsigned int u, const unsigned int v,
				     Matrix &m);

template <class Matrix, class Vector>
inline void matrix_set_vector_column(const Vector &vect,
				     const unsigned int u, const unsigned int v,
				     Matrix &m);

template <class Matrix, class Vector>
inline void matrix_set_vector_row(const unsigned int length,
				  const Vector &vect,
				  const unsigned int u, const unsigned int v,
				  Matrix &m);

template <class Matrix, class Vector>
inline void matrix_set_vector_row(const unsigned int length,
				  const Vector &vect,
				  const unsigned int u, const unsigned int v,
				  Matrix &m);

/*
 * Operations
 *
 */
template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_add(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c);

template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_add(const Matrix1 &a, const Matrix2 &b, Matrix3 &c);
      
template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_sub(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c);

template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_sub(const Matrix1 &a, const Matrix2 &b, Matrix3 &c);
    
template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul(const unsigned int n_rows, const unsigned int n_columns, const unsigned  int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c);
      
template <class T, class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul_T(const T* p, const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c);
      
template <class T, class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul_strip_T(const T* p, const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c, const int K=100);
      
template <class Matrix, class Vector1, class Vector2>
inline void matrix_mul_vector(const unsigned int n_rows, const unsigned int n_columns, const Matrix &a, const Vector1 &b, Vector2 &c);

template <class Matrix, class Vector1, class Vector2>
inline void matrix_mul_vector(const Matrix &a, const Vector1 &b, Vector2 &c);

template <class Matrix1, class T, class Matrix2>
inline void matrix_mul_k(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const T &k, Matrix2 &c); 

template <class Matrix>
int matrix_pivot_non_nul_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line);

template <class Matrix>
int matrix_pivot_greatest_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line);

template <class Matrix>
int matrix_pivot_nearest_one_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line);

template <class T, class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_Gauss_T(const T* p, const unsigned int n, const  Matrix &a, const Vector1 & b, Vector2 & x );

template <class T, class Matrix, class Vector1, class Vector2, class Matrix2>
inline void matrix_resolve_Gauss_T(const T *p, const unsigned int n, const Matrix &a, const  Vector1 & b, Vector2 & B, Matrix2 &temp);
	    
template <class T, class Matrix1, class Matrix2, class Matrix3, class Vector>
inline void matrix_inverse_Gauss_T(const T *p, const unsigned int n, Matrix1 &a, Matrix2 &inv_a, Vector &permutations, Matrix3 &temp);

// Find L U = A (L is lower diagonal, U is upper diagonal)
template <class Matrix1, class Matrix2, class Matrix3, class Vector_int>
inline void matrix_find_LU(const int n, const Matrix1 &a, Matrix2 &L, Matrix3 &U, Vector_int &t);

// Resolve U X = B (U is upper diagonal)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_upper(const int n, const Matrix &U, const Vector1 &b, Vector2 &x);
    
// Resolve L X = B (L is lower diagonal)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_lower(const int n, const Matrix &L, const Vector1 &b, Vector2 &x);
    
// Resolve L U X = B (U is upper diagonal, L is lower diagonal)
// temp must be a vector of dimension n)
template <class T,class Matrix1, class Matrix2, class Vector_int, class Vector1, class Vector2, class Vector3>
inline void matrix_resolve_LU_T(const T *p,
				const int n,
				const Matrix1 &L,
				const Matrix2 &U,
				const Vector_int &t,
				const Vector1 &b,
				Vector2 &x,
				Vector3 &temp);

#define macro_matrix_resolve_LU(T, n,L,U,t,b,x,temp) matrix_resolve_LU_T( (const T*)0, n, L, U, t, b, x, temp)
#define macro_matrix_mul(T,n_rows,n_columns,other_n_columns,a,b,c) matrix_mul_T((const T*) 0, n_rows,n_columns,other_n_columns,a,b,c)	    
#define macro_matrix_mul_strip(T,n_rows,n_columns,other_n_columns,a,b,c,K) matrix_mul_strip_T((const T*) 0, n_rows,n_columns,other_n_columns,a,b,c,K)
#define macro_matrix_resolve_Gauss(T,n,a,b,B) matrix_resolve_Gauss_T((const T*)0,n,a,b,B)
#define macro_matrix_resolve_Gauss_Faster(T, n, a, b, B, temp) matrix_resolve_Gauss_T( (const T*)0 ,n, a, b ,B, temp)
#define macro_matrix_inverse_Gauss(T, n, a, inv_a, permutations, temp) matrix_inverse_Gauss_T( (const T*)0, n, a, inv_a, permutations,temp)

#include "mat_gen.hpp"

#endif
