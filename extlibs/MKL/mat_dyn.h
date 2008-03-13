#ifndef __DYNAMIC_MATRIX_H_
#define __DYNAMIC_MATRIX_H_

//#include "portable_iostream.h"
//#include "portable_bool.h"

template <class T>
class Dynamic_Matrix
{
      bool dont_destroy;
	 
      void create();
      void copy(const Dynamic_Matrix<T> & other );
      void destroy();
	    
   public:
      T *m;

      unsigned int rows, columns;
	    
      inline Dynamic_Matrix();
      inline Dynamic_Matrix(unsigned int rows, unsigned int columns, T* data);
      inline Dynamic_Matrix(unsigned int rows, unsigned int columns);
      inline Dynamic_Matrix(const Dynamic_Matrix<T> & other);
      inline ~Dynamic_Matrix();
	    
      inline void resize(int rows, int columns);
	    
      inline Dynamic_Matrix<T> &operator = (const Dynamic_Matrix<T> & other);
	    
      inline T* operator[](const unsigned int index);
      inline const T* operator[](const unsigned int index)const;
};
      
template<class T> ostream & operator <<(ostream & out, const Dynamic_Matrix<T>  & other );

template<class T> inline unsigned int matrix_rows(const Dynamic_Matrix<T> & other);

template<class T> inline unsigned int matrix_columns(const Dynamic_Matrix<T> & other);

#include "mat_dyn.hpp"

#endif
