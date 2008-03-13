#ifndef __GENERIC_VECTOR_H_
#define __GENERIC_VECTOR_H_

template <class Vector>
std::ostream & vector_put(std::ostream &, unsigned int length, Vector &a);
	    
template <class Vector>
void vector_null(const unsigned int length, Vector &a);

template <class Vector>
void vector_null(Vector &a);

template <class Vector, class T>
void vector_fill(const unsigned int length, const T k, Vector &a);

template <class Vector1, class Vector2>
void vector_copy(const unsigned int length, const Vector1 &a, Vector2 &b);

template <class Vector1, class Vector2>
void vector_copy(const Vector1 &a, Vector2 &b);

template <class Vector1, class Vector2, class Vector3>
void vector_add(const unsigned int length, const Vector1 & a,const Vector2 & b, Vector3 &c);

template <class Vector1, class Vector2, class Vector3>
void vector_add(const Vector1 & a,const Vector2 & b, Vector3 &c);

template <class Vector1, class Vector2, class Vector3, class Vector4>
void vector_add3(const unsigned int length, const Vector1 & a,const Vector2 & b,const Vector3 &c, Vector4 &d);

template <class Vector1, class Vector2, class Vector3, class Vector4>
void vector_add3(const Vector1 & a,const Vector2 & b,const  Vector3 &c, Vector4 &d);

template <class Vector1, class Vector2, class Vector3>
void vector_sub(const unsigned int length, const Vector1 & a,const Vector2 & b, Vector3 &c);

template <class Vector1, class Vector2, class Vector3>
void vector_sub(const Vector1 & a,const Vector2 & b, Vector3 &c);

template <class Vector1, class T, class Vector2, class Vector3>
void vector_add_scale(const unsigned int length, const Vector1 & a, const T & k, const Vector2 & b, Vector3 &c);

template <class Vector1, class T, class Vector2, class Vector3>
void vector_add_scale(const Vector1 & a, const T & k, const Vector2 & b, Vector3 &c);

template <class Vector1, class T1, class Vector2, class T2, class Vector3, class Vector4>
void vector_add3_scale(const unsigned int length, const Vector1 & a, const T1 & k1, const Vector2 & b,
		       const T2 & k2, const Vector3 & c, Vector4 &d);

template <class Vector1, class T1, class Vector2, class T2, class Vector3, class Vector4>
void vector_add3_scale(const Vector1 & a, const T1 & k1, const Vector2 & b,
		       const T2 & k2, const Vector3 & c, Vector4 &d);

template <class Vector1, class T, class Vector3>
void vector_add_k(const unsigned int length, const Vector1 & a,const T & k, Vector3 &c);

template <class Vector1, class T, class Vector3>
void vector_mul_k(const unsigned int length, const Vector1 & a,const T & k, Vector3 &c);

template <class Vector1, class T, class Vector3>
void vector_mul_k(const Vector1 & a,const T & k, Vector3 &c);

template <class T, class Vector1, class Vector2>
void vector_scalar_product(const unsigned int length, const Vector1 & a,const Vector2 & b, T &scalar);

template <class T, class Vector1, class Vector2>
void vector_scalar_product(const Vector1 & a,const Vector2 & b, T &scalar);

template <class T, class Vector1, class Vector2>
T vector_scalar_product_T(const T* null, const Vector1 & a,const Vector2 & b);

template <class Vector1, class Vector2, class Vector3>
void vector_cross_product(const Vector1 & a,const Vector2 & b, Vector3 &c);

template <class T, class Vector>
void vector_norm_square(const unsigned int size, const Vector &a, T &scalar);

template <class T, class Vector>
void vector_norm(const unsigned int size, const Vector &a, T &scalar);

template <class T, class Vector>
void vector_normalize_T(const T*p, const unsigned int size, const Vector &a);

template <class T, class Vector>
void vector_normalize_T(const T*p, const Vector &a);

template <class Vector>
unsigned int vector_maximum(const unsigned int size, const Vector &v);

#define macro_vector_normalize_(T,size,a)   vector_normalize_T( (const T*)0, size, a)
#define macro_vector_normalize(T,a)   vector_normalize_T( (const T*)0, a)
#define macro_vector_scalar_product(T,a,b) vector_scalar_product_T( (const T*)0, a, b)

#include "vect_gen.hpp"

#include "op_selection.h"

#endif
