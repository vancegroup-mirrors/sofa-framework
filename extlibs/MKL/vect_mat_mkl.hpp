#include "vect_dyn.h"
#include "mat_dyn.h"
#include "mkl_cblas.h"

template <class T>
void matrix_mulMkl(const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Dynamic_Matrix<T> &a, const Dynamic_Matrix<T> &b, Dynamic_Matrix<T> &c)
{
	int             lda, ldb, ldc;
  double          alpha = 1.0, beta = 1.0;
  int             rmaxa, cmaxa, rmaxb, cmaxb, rmaxc, cmaxc;
  T *a1 = NULL, *b1 = NULL, *c1 = NULL;
	int i;
  CBLAS_ORDER     order = CblasRowMajor; //CblasColMajor
  CBLAS_TRANSPOSE transA = CblasNoTrans;
  CBLAS_TRANSPOSE transB = CblasNoTrans;

  rmaxa = n_rows + 1;
  cmaxa = n_columns;
  rmaxb = n_columns + 1;
  cmaxb = other_n_columns;
  rmaxc = n_rows + 1;
  cmaxc = other_n_columns;

  lda = cmaxa;
  ldb = cmaxb;
  ldc = cmaxc;

	// inutile de recopier a1, b1 cf matrices non-surdimensionnées (!= spore)
   /// c1 doit être à 0 car la résultat est rajouté à c1

  a1 = (T*) malloc (rmaxa*cmaxa*sizeof(T));
  b1 = (T*) malloc (rmaxb*cmaxb*sizeof(T));
  c1 = (T*) calloc (rmaxc*cmaxc, sizeof(T));

  if (a1 && b1) {
 	for (i = 0; i < n_rows; i++) 
		memcpy (a1 + i * n_columns, a.m + (i * a.columns), n_columns * sizeof(T));
	for (i = 0; i < n_columns; i++) 
		memcpy (b1 + i * other_n_columns, b.m + (i * b.columns), other_n_columns * sizeof(T));
  }
	
  cblas_dgemm(order, transA, transB, n_rows, other_n_columns, n_columns, alpha,
							a1, lda, b1, ldb, beta, c1, ldc); 
	
  for (i = 0; i < n_rows; i++)
	memcpy (c.m + i * b.columns , c1 + (i * other_n_columns), other_n_columns * sizeof(T));
	
  if (a1)
	free (a1);
  if (b1)
	free (b1);
  if (c1)
 	free (c1);
}

template <class T>
void matrix_mul_vectorMkl(const unsigned int n_rows, const unsigned int n_columns, const Dynamic_Matrix<T> &a, const Dynamic_Vector<T> &b, Dynamic_Vector<T> &c, int paramTransA = MAT_NO_TRANSPOSE)
{
  int lda;
  double alpha = 1.0, beta = 1.0;
  int rmaxa, cmaxa, incx = 1, incy = 1;

  CBLAS_ORDER     order = CblasRowMajor; 
  CBLAS_TRANSPOSE trans = (paramTransA == MAT_TRANSPOSE) ? CblasTrans : CblasNoTrans;

  rmaxa = a.rows;
  cmaxa = a.columns;

  lda = cmaxa; // car rowmajor...

  memset (c.v, 0, sizeof(T) * c.length);

  cblas_dgemv(order, trans, n_rows, n_columns, alpha, a.m, lda, b.v, incx,
              beta, c.v, incy);
}

/* fonction : b<- a * scaleFactor + b */
template <class T>
void vector_add_scaleMkl( int length, double scaleFactor, Dynamic_Vector<T> &a, const Dynamic_Vector<T> &b) {
  REQUIRE(vector_length(a) >= length);
  REQUIRE(vector_length(b) >= length);
  cblas_daxpy (length, scaleFactor, a.v, 1, b.v, 1);
}

template <class T>
void vector_copyMkl(int length, const Dynamic_Vector<T> &src, Dynamic_Vector<T> &dst) {
  REQUIRE(vector_length(src) >= length);
  REQUIRE(vector_length(dst) >= length);
  cblas_dcopy (length, src.v, 1, dst.v, 1);
}

template <class T>
void vector_dot_productMkl (const unsigned int length, const Dynamic_Vector<T> &a, const Dynamic_Vector<T> &b, double &res) {
  res = cblas_ddot (length, a.v, 1, b.v, 1);
} 

template <class T>
void vector_normMkl (const unsigned int length, const Dynamic_Vector<T> &a, double &norm) {
  norm = cblas_dnrm2 (length, a.v, 1);
}
