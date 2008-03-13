#ifndef __DYNAMIC_VECTOR_H_
#define __DYNAMIC_VECTOR_H_

#include <iostream>

template<class T>
class Dynamic_Vector
{
	  
   private:
      inline void create();
      inline void copy(const Dynamic_Vector<T> & other );
      inline void destroy();
	  
   public:
      T *v;
	  
      unsigned int length;
      unsigned int reserved_size;
	  
      inline Dynamic_Vector();
      inline Dynamic_Vector(unsigned int length);
      inline Dynamic_Vector(const Dynamic_Vector<T> & other);
      inline ~Dynamic_Vector();

	  
      inline unsigned int size() const;
      inline bool is_empty() const;
      inline void empty();
      inline unsigned int capacity();
      inline void reserve(unsigned int size);
      inline void resize(unsigned int size);

      inline void push_back(const T &other);
      inline T pop_back();
	  
      inline Dynamic_Vector<T> & operator = (const Dynamic_Vector<T> & other);

      inline T & operator[](const unsigned int index);
      inline const T & operator[](const unsigned int index) const;
};

template<class T>
std::ostream & operator <<(std::ostream & out, const Dynamic_Vector<T> & other);

template<class T> unsigned int vector_length(const Dynamic_Vector<T> & other);

#include "vect_gen.h"
#include "vect_dyn.hpp"

#endif
