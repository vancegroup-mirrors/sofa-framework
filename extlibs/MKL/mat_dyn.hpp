#include "mat_gen.h"

template<class T>
inline Dynamic_Matrix<T>::Dynamic_Matrix()
{
      Dynamic_Matrix::rows=0;
      Dynamic_Matrix::columns=0;
      dont_destroy=true;
      m=0;
}

template<class T>
inline Dynamic_Matrix<T>::Dynamic_Matrix(unsigned int size_row,unsigned  int size_column, T *data) 
{
    Dynamic_Matrix::rows=size_row;
    Dynamic_Matrix::columns=size_column;
    dont_destroy=true;
    m=data; 
}

template<class T>
inline Dynamic_Matrix<T>::Dynamic_Matrix(unsigned int size_row,unsigned  int size_column) 
{
    Dynamic_Matrix::rows=size_row;
    Dynamic_Matrix::columns=size_column;
    dont_destroy=false;
    create();
    
}

template<class T>
inline Dynamic_Matrix<T>::Dynamic_Matrix(const Dynamic_Matrix<T> &other)
{
    rows=other.rows;
    columns=other.columns;
    dont_destroy=false;
    copy(other);
}

template<class T>
inline void Dynamic_Matrix<T>::resize(int size_row, int size_column) 
{
  destroy();
  rows=size_row;
  columns=size_column;
  create();
}

template<class T>
Dynamic_Matrix<T> &Dynamic_Matrix<T>::operator = (const Dynamic_Matrix<T> & other)
{
    copy(other);
    return *this;
}


template<class T>
inline Dynamic_Matrix<T>::~Dynamic_Matrix()
{
      destroy();
}

template<class T>
inline void Dynamic_Matrix<T>::create()
{
    m=(T*)calloc((rows + 1)*(columns + 1), sizeof(T));
}

template<class T>
inline void Dynamic_Matrix<T>::copy(const Dynamic_Matrix<T> & other )
{
    if(&other !=this)
	{
	    if(rows*columns!=other.rows*other.columns)
		{
		    destroy();
		    rows=other.rows;
		    columns=other.columns;
		    dont_destroy=false;
		    create();
		}
	    unsigned int i;
	    for(i=0; i<rows; i++)
		{
		    T* t1=(*this)[i];
		    const T* t2=(other)[i];
		    unsigned int j;
		    for(j=0; j<columns;j++)
			t1[j]=t2[j];
		}	    
	}
}

template<class T>
inline void Dynamic_Matrix<T>::destroy()
{
      if(!dont_destroy)  free(m);
      m=0;
}

template<class T>
inline T* Dynamic_Matrix<T>::operator[](const unsigned int index)
{
	return m+index*columns;
}

template<class T>
inline const T* Dynamic_Matrix<T>::operator[](const unsigned int index) const
{
	return m+index*columns;
}

template<class T>
ostream & operator <<(ostream & out, const Dynamic_Matrix<T>  & other )
{
      return matrix_put(out,other.rows,other.columns, other);
}

template<class T> inline unsigned int matrix_rows(const Dynamic_Matrix<T> & other)
{
      return other.rows;
}

template<class T> inline unsigned int matrix_columns(const Dynamic_Matrix<T> & other)
{
      return other.columns;
}
