
template <class T>
void MultMatTransVec(Dynamic_Matrix<T> &A,Dynamic_Vector<T> &X, Dynamic_Vector<T>&AX,int size)
{
  for (int i=0;i<size;i++)
    {
      AX[i] = 0;
      for (int j=0;j<size;j++)
        AX[i] += A[j][i]*X[j];
    }
}

template <class T>
void Residu(Dynamic_Matrix<T>&A, Dynamic_Vector<T> &X, Dynamic_Vector<T>&B,
            Dynamic_Vector<T>&Residu, int size)
{
  Dynamic_Vector<T> tmpB = Dynamic_Vector<T>(size);
#ifdef USE_MKL
  matrix_mul_vectorMkl(size, size, A, X, Residu);

  vector_copyMkl (size, B, tmpB);
  vector_add_scaleMkl(size, -1.0, Residu, tmpB);
  vector_copyMkl (size, tmpB, Residu);

#else
  matrix_mul_vector (size, size, A, X, Residu);
  vector_sub (B, Residu, Residu);
#endif
}

template <class T>
BiCG<T>::BiCG(int _size, double _tolErr) {
  size=_size;
  tolErr = _tolErr;
  P = Dynamic_Vector<T>(size);
  R = Dynamic_Vector<T>(size);
  R0 = Dynamic_Vector<T>(size);
  P0 = Dynamic_Vector<T>(size);
  Q = Dynamic_Vector<T>(size);
  Q0 = Dynamic_Vector<T>(size);
  tmpVect = Dynamic_Vector<T>(size);
}

template <class T>
BiCG<T>::~BiCG(void) {
}

template <class T>
void BiCG<T>::setTolErr (double newTolErr) {
	tolErr = newTolErr;
}

#ifdef USE_MKL
template<class T>
int BiCG<T>::Resol (Dynamic_Matrix<T> &A, Dynamic_Vector<T> &X, Dynamic_Vector<T> &B)
{
  double rho_1, rho_2, alpha, beta, tmp;
  int NbIter=0;
  double normB, normR;
  
  Residu(A, X, B, R,size);
  vector_normMkl (size, B, normB);
  vector_copyMkl (size, R, R0);
  
  if (normB == 0.0)
    normB = 1.0;
		
  vector_normMkl (size, R, normR);

  while ((normR / normB) >= tolErr) {
    vector_dot_productMkl (size, R0, R, rho_1);
    if (rho_1 == 0) {
	    break;
    }
    if (NbIter == 0) {
      vector_copyMkl (size, R, P);
      vector_copyMkl (size, R0, P0);
    } else {
      beta = rho_1 / rho_2;
      /* P <- beta * P + R */
      vector_copyMkl (size, R, tmpVect);
      vector_add_scaleMkl (size, beta, P, tmpVect);
      vector_copyMkl (size, tmpVect, P);
      /* P0 <- beta * P0 + R0 */
      vector_copyMkl (size, R0, tmpVect);
      vector_add_scaleMkl (size, beta, P0, tmpVect);
      vector_copyMkl (size, tmpVect, P0);			
    }
    matrix_mul_vectorMkl(size, size, A, P, Q);
    matrix_mul_vectorMkl(size, size, A, P0, Q0, MAT_TRANSPOSE);
    //vector_dot_productMkl (size, P0, Q, tmp);
    vector_dot_productMkl (size, P0, Q, tmp);
    if (tmp == 0.0)
      break;
    else
      alpha = rho_1 / tmp; 
    /*		X += alpha * P; */
    vector_add_scaleMkl (size, alpha, P, X);
    /* R -= alpha * Q; */
    vector_add_scaleMkl (size, -alpha, Q, R);
    /* R0 -= alpha * Q0; */
    vector_add_scaleMkl (size, -alpha, Q0, R0);
    rho_2 = rho_1;
    vector_normMkl (size, R, normR);		
    NbIter++;
  }
  return NbIter; 
}

#else


template<class T> 
int BiCG<T>::Resol( Dynamic_Matrix<T> &A, Dynamic_Vector<T> &X, Dynamic_Vector<T> &B)
{
  double rho_1, rho_2, alpha, beta, tmp;
  int NbIter=0;
  double normB, normR;
  int i;
  
  
  Residu(A, X, B, R,size);
  vector_norm (size, B, normB);
  vector_copy (size, R, R0);
  if (normB == 0.0)
    normB = 1.0;
		
  vector_norm (size, R, normR);
  while ((normR / normB) >= tolErr) {
    vector_scalar_product (R0, R, rho_1);
    if (rho_1 == 0) {
      break;
    }
    if (NbIter == 0) {
      vector_copy (size, R, P);
      vector_copy (size, R0, P0);
    } else {
      beta = rho_1 / rho_2;
      /* P <- beta * P + R */
      vector_add_scale (size, R, beta, P, P);
      /* P0 <- beta * P0 + R0 */
      vector_add_scale (size, R0, beta, P0, P0);			
    }
    matrix_mul_vector(size, size, A, P, Q);
    MultMatTransVec (A, P0, Q0, size);
    vector_scalar_product (size, P0, Q, tmp);
    
    if (tmp == 0.0)
      break;
    else
      alpha = rho_1 / tmp; 
    /*		X += alpha * P; */
    vector_add_scale (size, X, alpha, P, X);	
    /* R -= alpha * Q; */
    vector_add_scale (size, R, -alpha, Q, R);
    /* R0 -= alpha * Q0; */
    vector_add_scale (size, R0, -alpha, Q0, R0);
    rho_2 = rho_1;
    vector_norm (size, R, normR);
    NbIter++;
  }
  return NbIter; 
}

#endif
