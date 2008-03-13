#ifndef _BICG_H_
#define _BICG_H_

#include "vect_dyn.h"
#include "mat_dyn.h"
#include "vect_mat_mkl.h"

template <class T>
void MultMatTransVec(Dynamic_Matrix<T> &A,Dynamic_Vector<T> &X, Dynamic_Vector<T>&AX,int size);

template <class T>
void Residu(Dynamic_Matrix<T>&A, Dynamic_Vector<T> &X, Dynamic_Vector<T>&B,
            Dynamic_Vector<T>&Residu, int size);

template <class T>
class BiCG
{
protected:
  int size;
  double tolErr;
  Dynamic_Vector<T> P, R, R0, P0, Q, Q0, tmpVect;
	
public:
  BiCG(int _size, double _tolErr);
  ~BiCG(void);	
	
  void setTolErr (double newTolErr);
  int Resol (Dynamic_Matrix<T> &A, Dynamic_Vector<T> &X, Dynamic_Vector<T> &B);
};

#include "bicg.hpp"

#endif /* _BICG_H_ */
