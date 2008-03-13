#include "vect_gen.h"
#include "vect_dyn.h"
#include "mat_dyn.h"
//#include "portable_helpers.h"
#include "debug.h"

template <class Matrix>
ostream & matrix_put(ostream & out, const unsigned int n_rows, const unsigned int n_columns, Matrix &m)
{
      unsigned int i,j;
      out << "{" << endl;
      
      for(i=0; i<n_rows; i++)
	{
	    for(j=0; j<n_columns;j++)
		out << m[i][j] << " ";
	    out << endl;
	}
    out <<"}" <<endl;
    
    return out;
}


template <class Matrix>
inline void matrix_null(const unsigned int n_rows, const unsigned int n_columns, Matrix &a)
{
      unsigned int i,j;
      for(i=0; i<n_rows; i++)
	    for(j=0; j<n_columns; j++)
	    a[i][j]=0;
}

template <class Matrix>
inline void matrix_null(Matrix &a)
{
      matrix_null(matrix_rows(a),matrix_columns(a),a);
}


template <class Matrix>
inline void matrix_identity(const unsigned int n_rows, const unsigned int n_columns, Matrix &a)
{
      unsigned int i,j;
      for(i=0; i<n_rows; i++)
	    for(j=0; j<n_columns; j++)
		  a[i][j]=0;
      for(i=0; i<n_rows; i++)
	    a[i][i]=1;
}

template <class Matrix>
inline void matrix_identity(Matrix &a)
{
      REQUIRE(matrix_rows(a)==matrix_columns(a));
      matrix_identity(matrix_rows(a),matrix_columns(a),a);
}

template < class T, class Matrix>
inline void matrix_rotation_x(Matrix &m, T& angle)
{
      matrix_identity(m);
      
      T ss=sin(angle);
      m[2][1]=ss;
      m[1][2]=-ss;
      
      T cs=cos(angle);
      m[1][1]=cs;
      m[2][2]=cs;
}

template < class T, class Matrix>
inline void matrix_rotation_y(Matrix &m, T& angle)
{
      matrix_identity(m);
      
      T ss=sin(angle);
      m[0][2]=ss;
      m[2][0]=-ss;
      
      T cs=cos(angle);
      m[2][2]=cs;
      m[0][0]=cs;
}

template < class T, class Matrix>
inline void matrix_rotation_z(Matrix &m, T& angle)
{
      matrix_identity(m);
      
      T ss=sin(angle);
      m[1][0]=ss;
      m[0][1]=-ss;
      
      T cs=cos(angle);
      m[0][0]=cs;
      m[1][1]=cs;
}

template <class Matrix1, class Matrix2>
inline bool matrix_is_equal(const unsigned int n, const unsigned int m, const Matrix1 &a, const Matrix2 &b)
{
     unsigned int i=0,j=0;
     for(i=0; i<n; i++)
	for(j=0; j<m; j++)
	    if(a[i][j]!=b[i][j]) return false;
     return true;
}

template <class Matrix, class T>
inline void matrix_fill(const unsigned int n_rows, const unsigned int n_columns, const T &k, Matrix &a)
{
       unsigned int i,j;
    for(i=0; i<n_rows; i++)
	for(j=0; j<n_columns; j++)
	    a[i][j]=k; 
}

template <class Matrix1, class Matrix2>
void matrix_copy(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix2 &b)
{
      unsigned int i,j;
    for(i=0; i<n_rows; i++)
	for(j=0; j<n_columns; j++)
	    b[i][j]=a[i][j];
}

template <class Matrix1, class Matrix2>
void matrix_copy_to_array(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix2 b)
{
      unsigned int i,j;
    for(i=0; i<n_rows; i++)
	for(j=0; j<n_columns; j++)
	    b[i][j]=a[i][j];
}

template <class Matrix1, class Matrix3>
void matrix_transpose(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, Matrix3 &c)
{
      unsigned int i,j;
    for(i=0; i<n_rows;i++)
	for(j=0; j<n_columns; j++)
	    c[j][i]=a[i][j];
}

template <class Matrix1, class Matrix3>
inline void matrix_transpose(const Matrix1 &a, Matrix3 &c)
{
      REQUIRE(matrix_rows(a)==matrix_columns(c));
      REQUIRE(matrix_columns(a)==matrix_rows(c));
      matrix_transpose(matrix_rows(a),matrix_columns(a),a,c);
}



template <class Matrix1, class Matrix2>
inline void matrix_set(const unsigned int n_rows, const unsigned int n_columns,
		       const Matrix1& sub_matrix,
		       const unsigned int u, const unsigned int v,
		       Matrix2 &m)
{
      unsigned int i,j;
      for(i=0; i<n_rows;i++)
	for(j=0; j<n_columns; j++)
	      m[u+i][v+j]=sub_matrix[i][j];
}


template <class Matrix1, class Matrix2>
inline void matrix_set(const Matrix1& sub_matrix,
		       const unsigned int i, const unsigned int j,
		       Matrix2 &m)
{
      matrix_set( matrix_rows(sub_matrix),matrix_columns(sub_matrix), sub_matrix,i,j,m);
}

template <class Matrix, class Vector>
inline void matrix_set_vector_column(const unsigned int length,
				     const Vector &vect,
				     const unsigned int u, const unsigned int v,
				     Matrix &m)
{
      unsigned int i;
      for(i=0;i<length;i++)
	    m[u+i][v]=vect[i];
}

template <class Matrix, class Vector>
inline void matrix_set_vector_column(const Vector &vect,
				     const unsigned int u, const unsigned int v,
				     Matrix &m)
{
      matrix_set_vector_column(vector_length(vect), vect, u, v, m);
}


template <class Matrix, class Vector>
inline void matrix_set_vector_row(const unsigned int length,
				  const Vector &vect,
				  const unsigned int u, const unsigned int v,
				  Matrix &m)
{
      unsigned int i;
      for(i=0;i<length;i++)
	    m[u][v+i]=vect[i];
}

template <class Matrix, class Vector>
inline void matrix_set_vector_row(const Vector &vect,
				  const unsigned int u, const unsigned int v,
				  Matrix &m)
{
      matrix_set_vector_row(vector_length(vect), vect, u, v, m);
}

template <class Matrix1, class Matrix2, class Matrix3>
void matrix_add(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c)
{
      unsigned int i,j;
    for(i=0; i<n_rows; i++)
	for(j=0; j<n_columns; j++)
	    c[i][j]=a[i][j]+b[i][j];
}

template <class Matrix1, class Matrix2, class Matrix3>
void matrix_add(const Matrix1 &a, const Matrix2 &b, Matrix3 &c)
{
      REQUIRE(matrix_rows(a)==matrix_rows(b));
      REQUIRE(matrix_rows(a)==matrix_rows(c));
      REQUIRE(matrix_columns(a)==matrix_columns(b));
      REQUIRE(matrix_columns(a)==matrix_columns(c));
      matrix_add(matrix_rows(a),matrix_columns(b),a,b,c);
}


template <class Matrix1, class Matrix2, class Matrix3>
void matrix_sub(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c)
{
      unsigned int i,j;
    for(i=0; i<n_rows; i++)
	for(j=0; j<n_columns; j++)
	    c[i][j]=a[i][j]-b[i][j];
}

template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_sub(const Matrix1 &a, const Matrix2 &b, Matrix3 &c)
{
      REQUIRE(matrix_rows(a)==matrix_rows(b));
      REQUIRE(matrix_rows(a)==matrix_rows(c));
      REQUIRE(matrix_columns(a)==matrix_columns(b));
      REQUIRE(matrix_columns(a)==matrix_columns(c));
      matrix_sub(matrix_rows(a),matrix_columns(b),a,b,c);
}

   
template <class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul(const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c)
{
    unsigned int i,j,k;  
    for(i=0;i<n_rows; i=i+1)
	for(j=0;j<other_n_columns;j=j+1) {
	      c[i][j]=a[i][0]*b[0][j];
	      for(k=1; k<n_columns; k=k+1)
		    c[i][j]+=a[i][k]*b[k][j];
	}
}

template <class T, class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul_T(const T* p, const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c )
{
      unsigned int i,j,k;
      T tmp;

      for(i=0;i<n_rows;i=i+1) {
	    for(j=0; j<n_columns; j=j+1) c[i][j]=0;
	    for(k=0; k<n_columns; k=k+1) {
		  tmp = a[i][k];
		  for(j=0;j<n_columns;j=j+1)
			c[i][j] += tmp*b[k][j];
	    }
      }
}

template <class T, class Matrix1, class Matrix2, class Matrix3>
inline void matrix_mul_strip_T(const T* p,const unsigned int n_rows, const unsigned int n_columns, const unsigned int other_n_columns, const Matrix1 &a, const Matrix2 &b, Matrix3 &c, const unsigned int K)
{
      unsigned int i,j,k,ii,kk;
      T tmp;

      for(ii=0;ii<n_rows;ii=ii+K)
	    for(kk=0;kk<n_columns;kk+kk+K) {
		  for(i=ii;i<ii+K;i=i+1)
			for(j=0;j<other_n_columns;j=j+1)
			      C[i][j]=0;
		  for(i=ii;i<ii+K;i=i+1)
			for(k=kk; k<kk+K; k=k+1) {
			      tmp= A[i][k];
			      for(j=0;j<other_n_columns;j=j+1)
				    C[i][j] = C[i][j] + tmp*b[k][j];
			}
	    }
}

template <class Matrix, class Vector1, class Vector2>
void matrix_mul_vector(const unsigned int n_rows, const unsigned int n_columns, const Matrix &a, const Vector1 &b, Vector2 &c)
{
      unsigned int i,j;
      for(i=0; i<n_rows; i++) {
	    c[i] = a[i][0]*b[0];
	    for(j=1; j<n_columns; j++)
		  c[i] += a[i][j]*b[j];
      }
}

template <class Matrix, class Vector1, class Vector2>
inline void matrix_mul_vector(const Matrix &a, const Vector1 &b, Vector2 &c)
{
      REQUIRE(matrix_columns(a)==vector_length(b));
      REQUIRE(matrix_rows(a)==vector_length(c));
      matrix_mul_vector(matrix_rows(a),matrix_columns(a),a,b,c);
}


template <class Matrix1, class T, class Matrix2>
void matrix_mul_k(const unsigned int n_rows, const unsigned int n_columns, const Matrix1 &a, const T &k, Matrix2 &c)
{
     unsigned int i,j;  
     for(i=0; i<n_rows; i++)
	   for(j=0; j<n_columns;j++)
		 c[i][j]=a[i][j]*k;
}

template <class Matrix>
int matrix_pivot_non_nul_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line)
{
      unsigned int j;
      for(j=starting_line; j<size; j++) {
	    if(A[j][starting_line]!=0.0)
		  return j;
      }
      //EXIT("PIVOT NOT FOUND");
      //return -1;
      WARNING("PIVOT NOT FOUND"<<endl);
      return starting_line;
}

template <class Matrix>
int matrix_pivot_greatest_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line)
{
      int index=starting_line;
      unsigned int j;
      for(j=starting_line; j<size; j++) {
	    if( fabs(A[j][starting_line]) > fabs(A[index][starting_line]) )
		  index=j;
      }
      if (A[index][starting_line] == 0.0 ) EXIT("PIVOT NOT FOUND"<<endl);
      return index;
}

template <class Matrix>
int matrix_pivot_nearest_one_lower(const unsigned int size, const Matrix &A ,unsigned int starting_line)
{
      int index=starting_line;
      unsigned int j;
      for(j=starting_line; j<size; j++) {
	    if( fabs( fabs(A[j][starting_line]) - 1.0 ) < fabs(fabs(A[index][starting_line]) - 1.0 ))
		  index=j;
      }
      if (A[index][starting_line] == 0.0 ) EXIT("PIVOT NOT FOUND"<<endl);
      return index;   
}


template <class T, class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_Gauss_T(const T* p, const unsigned int n, const Matrix &a, const  Vector1 & b, Vector2 & B )
{
    Dynamic_Matrix< T > temp(n,n);
    macro_matrix_resolve_Gauss_Faster(T, n, a, b, B, temp);
}

// Find L U = A
// L is lower diagonal
// U is upper diagonal
template <class Matrix1, class Matrix2, class Matrix3, class Vector_int>
inline void matrix_find_LU(const unsigned int n, const Matrix1 &a, Matrix2 &L, Matrix3 &U, Vector_int &t)
{
      unsigned int i,j,k;
      
    matrix_copy(n,n,a,U);
    matrix_identity(n,n,L);
    for(i=0;i<n;i++) t[i]=i;
    
    for(k=0;k<n-1;k++)
	{
	    //recherche un pivot non null
	    unsigned int line=matrix_pivot_non_nul_lower(n,U,k);
	    
	    if (line != k)
		{
		    for(j=0; j<n; j++) portable_swap(U[k][j],U[line][j]);
		    portable_swap(t[k],t[line]);
		}

	    
	    // LU
	    for(i=k+1;i<n;i++)
		L[i][k]=U[i][k]/U[k][k];
	    for(i=k+1;i<n;i++)
		for(j=k+1;j<n;j++)
		    U[i][j]-=L[i][k]*U[k][j];
	    for(j=0;j<=k;j++)
		U[k+1][j]=0;
	}
}


// Resolve U X = B,
// U is upper diagonal
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_upper(const unsigned int n, const Matrix &U, const Vector1 &b, Vector2 &x)
{
      int i;
      unsigned int j;
    for(i=n-1;i>=0;i--)
	{
	    x[i]=b[i];
	    for(j=i+1; j<n; j++)
		{
		    x[i]-=U[i][j]*x[j];
		}
	    x[i]/=U[i][i];
	}
}


// Resolve L X = B
// L is lower diagonal
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_lower(const unsigned int n, const Matrix &L, const Vector1 &b, Vector2 &x)
{
      unsigned int i,j;
    for(i=0;i<n;i++)
	{
	    x[i]=b[i];
	    for(j=0; j<i; j++)
		{
		    x[i]-=L[i][j]*x[j];
		}
	    x[i]/=L[i][i];
	}
}

template <class T, class Matrix, class Vector1, class Vector2, class Matrix2>
inline void matrix_resolve_Gauss_T(const T*, const unsigned int n, const Matrix &a, const  Vector1 & b, Vector2 & B, Matrix2 &temp)
{
    unsigned int i,j,k;
   
    matrix_copy(n,n,a,temp);
    vector_copy(n,b,B);    
	
    //Triangulation inferieure
    for(i=0; i<n; i++)
	{
	    //recherche un pivot non nul 
	    unsigned int line=matrix_pivot_non_nul_lower(n,temp,i);
	    
	     if (line != i)
 		{
 		    for(j=0; j<n; j++) portable_swap(temp[i][j],temp[line][j]);
 		    portable_swap(B[i],B[line]);
 		}
	    
	    //transforme la diagonale en 1
	    T diag=1.0/temp[i][i];
	    for(k=i; k<n; k++) temp[i][k]*=diag;
	    B[i]*=diag;
	    
	    //applique le pivot sur les lignes inferieures a i
	    for(j=i+1;j<n;j++)
		{
		    T pivot=temp[j][i];
		    for(k=i;k<n;k++) temp[j][k]-=pivot*temp[i][k];
		    B[j]-=pivot*B[i];
		}
	}

    //Diagonalise
    for(i=n-2; (signed)i>=0; i--)
	    for(k=i+1;k<n;k++)
	      B[i]-=temp[i][k]*B[k];
}


template <class T, class Matrix1, class Matrix2, class Matrix3, class Vector>
inline void matrix_inverse_Gauss_T(const T *, const unsigned int n, Matrix1 &a, Matrix2 &inv_a, Vector &permutations, Matrix3 &temp)
{
      unsigned int i,j,k;
      matrix_identity(n,n,temp);
      matrix_copy(n,n,a,inv_a);
      for(k=0; k<n; k++) permutations[k]=k;
      for(i=0; i<n; i++) {
	    unsigned int p=matrix_pivot_non_nul_lower(n,inv_a,i);
	    if(p!=i) {
		  for(k=i; k<n; k++) portable_swap(inv_a[i][k],inv_a[p][k]);
		  for(k=0; k<n; k++) portable_swap(temp[i][k],temp[p][k]);
		  portable_swap(permutations[i],permutations[p]);
	    }
	    T diag= 1.0 / inv_a[i][i];
	    for(k=i; k<n; k++) inv_a[i][k]*=diag;
	    for(k=0; k<n; k++) temp[i][k]*=diag;
	    
	    for(j=i+1; j<n; j++) {
		  T pivot=inv_a[j][i];
		  for( k=i; k<n; k++) inv_a[j][k]-=pivot*inv_a[i][k];
		  for( k=0; k<n; k++) temp[j][k]-=pivot*temp[i][k];
	    }
      }

      for( i=n-1; i<n ; i-=1) {
	    for(j=0; j<i; j++) {
		  T pivot=inv_a[j][i];
		  for(k=j; k<=i; k++) inv_a[j][k]-=pivot*inv_a[i][k];
		  for(k=0; k<n; k++) temp[j][k]-=pivot*temp[i][k];
	    }
      }

    //   cout << "A" << a << endl;
//       cout << "inv_A (Must be Identity)" << inv_a << endl;
//       cout << "temp" << temp << endl;
//       cout << "permutations" <<  permutations << endl;
      
      for(i=0; i<n; i++)
	    for(j=0; j<n; j++)
		  inv_a[i][j]=temp[ permutations[i] ][j];

     //  cout << "inv_A (final)" << inv_a << endl;
}

template <class Matrix, class Vector>
inline void matrix_column(const unsigned int rows, const unsigned int columns, const Matrix &a, const unsigned int c, Vector &u) {
      for(unsigned int i=0;i<rows;i++) u[i]=a[i][c];
}

template <class Matrix, class Vector>
inline void matrix_column(const Matrix &a, const unsigned int c, Vector &u) {
      REQUIRE(matrix_rows(a)==vector_length(u));
      matrix_column(matrix_rows(a),matrix_columns(a),a,c,u);
}

template <class Matrix, class Vector>
inline void matrix_row(const unsigned int rows, const unsigned int columns, const Matrix &a, const unsigned int r, Vector &u) {
      for(int i=0;i<columns;i++) u[i]=a[r][i];
}

template <class Matrix, class Vector>
inline void matrix_row(const Matrix &a, const unsigned int r, Vector &u) {
      REQUIRE(matrix_columns(a)==vector_length(u));
      matrix_row(matrix_rows(a),matrix_columns(a),a,r,u);
}

/*******************************************
  LU Decomposition
*******************************************/

// Find L U = A
// L is lower diagonal
// U is upper diagonal
template <class Matrix1, class Matrix2, class Matrix3, class Vector_int>
inline void matrix_find_LU(const int n, const Matrix1 &a, Matrix2 &L, Matrix3 &U, Vector_int &t)
{    
    int i,j,k;
    matrix_copy(n,n,a,U);
    matrix_identity(n,n,L);
    for(i=0;i<n;i++) t[i]=i;
 
    for(k=0;k<n-1;k++){
	    //recherche un pivot non null
	    int line=matrix_pivot_non_nul_lower(n,U,(unsigned)k);
	    
	    if (line != k){
		    for(int j=0; j<n; j++) portable_swap(U[k][j],U[line][j]);
		    portable_swap(t[k],t[line]);
      }
	    
	    // LU
	    for(i=k+1;i<n;i++) L[i][k]=U[i][k]/U[k][k];
	    for(i=k+1;i<n;i++)
        for(j=k+1;j<n;j++)
		      U[i][j]-=L[i][k]*U[k][j];
	    for(j=0;j<=k;j++) U[k+1][j]=0;
    }
}


// Resolve U X = B,
// U is upper diagonal
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_upper(const int n, const Matrix &U, const Vector1 &b, Vector2 &x)
{
      int i,j;
    for(i=n-1;i>=0;i--)
	{
	    x[i]=b[i];
	    for(j=i+1; j<n; j++)
		{
		    x[i]-=U[i][j]*x[j];
		}
	    x[i]/=U[i][i];
	}
}


// Resolve L X = B
// L is lower diagonal
template <class Matrix, class Vector1, class Vector2>
inline void matrix_resolve_lower(const int n, const Matrix &L, const Vector1 &b, Vector2 &x)
{
      int i,j;
    for(i=0;i<n;i++)
	{
	    x[i]=b[i];
	    for(j=0; j<i; j++)
		{
		    x[i]-=L[i][j]*x[j];
		}
	    x[i]/=L[i][i];
	}
}

// Resolve L U X = B
// U is upper diagonal
// L is lower diagonal
template <class T, class Matrix1, class Matrix2, class Vector_int, class Vector1, class Vector2, class Vector3>
inline void matrix_resolve_LU_T(const T*,
			 const int n,
			 const Matrix1 &L,
			 const Matrix2 &U,
			 const Vector_int &t,
			 const Vector1 &b,
			 Vector2 &x,
			 Vector3 &temp)
{
      int i;
    for(i=0;i<n;i++) x[i]=b[t[i]];
    matrix_resolve_lower(n,L,x,temp);
    matrix_resolve_upper(n,U,temp,x);
}

