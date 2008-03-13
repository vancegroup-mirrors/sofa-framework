#ifndef __OP_GENERIC_H_
#define __OP_GENERIC_H_


template<class T> T generic_vector_scalar_product1(unsigned int size,T *a, T*b);
template<class T> T generic_vector_scalar_product2(unsigned int size,T *a, T*b);
template<class T> T generic_vector_scalar_product3(unsigned int size,T *a, T*b);
template<class T> T generic_vector_scalar_product4(unsigned int size,T *a, T*b);
template<class T> T generic_vector_norm_square(unsigned int size, T*a);
template<class T> void generic_vector_copy(unsigned int size, T*a, T*b);
template<class T> void generic_vector_mul_k(unsigned int size, T*a, T k, T*b);
template<class T> inline void generic_vector_add_mul_k(unsigned int size, T*a, T k, T*b, T*c);
template<class T> void generic_vector_neg(unsigned int size, T*a, T*b);

template<class T, unsigned int size>  void generic_vector_add_const_size(T*a, T*b, T*c);
template<class T, unsigned int size>  void generic_vector_sub_const_size(T*a, T*b, T*c);
template<class T, unsigned int size>  void generic_vector_mul_const_size(T*a, T*b, T*c);
	
template<class T> void generic_add(T*a, T*b, T*c);
template<class T> void generic_vector2_add(T*a, T*b, T*c);
template<class T> void generic_vector3_add(T*a, T*b, T*c);
template<class T> void generic_vector4_add(T*a, T*b, T*c);

template<class T> void generic_sub(T*a, T*b, T*c);
template<class T> void generic_vector2_sub(T*a, T*b, T*c);
template<class T> void generic_vector3_sub(T*a, T*b, T*c);
template<class T> void generic_vector4_sub(T*a, T*b, T*c);
	
template<class T> void generic_mul(T*a, T*b, T*c);
template<class T> void generic_vector2_mul(T*a, T*b, T*c);
template<class T> void generic_vector3_mul(T*a, T*b, T*c);
template<class T> void generic_vector4_mul(T*a, T*b, T*c);
			
template<class T> void generic_vector_add(unsigned int size, T*a, T*b, T*c);
template<class T> void generic_vector_sub(unsigned int size, T*a, T*b, T*c);
template<class T> void generic_vector_mul(unsigned int size, T*a, T*b, T*c);



#include "op_generic.hpp"

#endif
