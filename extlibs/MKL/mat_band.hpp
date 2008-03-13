/*******************************************
  LU Decomposition with band Matrix
*******************************************/

// Find L U = A (A band matrix of large l)
// L is lower diagonal (band matrix of large l/2 + 1)
// U is upper diagonal (band matrix of large l/2 + 1) 
template <class Matrix1, class Matrix2, class Matrix3, class Vector_int>
inline void matrix_band_find_LU(const int n,const int,const Matrix1 &a, Matrix2 &L, Matrix3 &U, Vector_int &t)
{    
    int i,j,k;
    matrix_copy(n,n,a,U);
    matrix_identity(n,n,L);
    for(i=0;i<n;i++) t[i]=i;
 
    for(k=0;k<n-1;k++){
	    //recherche un pivot non null
	    int line=matrix_pivot_non_nul_lower(n,U,(unsigned int)k);
	    
	    if (line != k){
		    for(j=0; j<n; j++) portable_swap(U[k][j],U[line][j]);
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
// U is upper diagonal (band of large k/2 + 1)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_band_resolve_upper(const int n,const int k,const Matrix &U, const Vector1 &b, Vector2 &x)
{
  int nbChiffreMax=1+k/2,indiceMax,i,j;

    for(i=n-1;i>=0;i--)
	{
	    x[i]=b[i];
      indiceMax=(i+nbChiffreMax>n?n:i+nbChiffreMax);
	    for(j=i+1; j<indiceMax; j++)
		{
		    x[i]-=U[i][j]*x[j];
		}
	    x[i]/=U[i][i];
	}
}
// Resolve L X = B
// L is lower diagonal (band of large k/2 + 1)
template <class Matrix, class Vector1, class Vector2>
inline void matrix_band_resolve_lower(const int n,const int k,const Matrix &L, const Vector1 &b, Vector2 &x)
{
  int nbChiffreMax=1+k/2,indiceMin,i,j;

  for(i=0;i<n;i++)
	{
	    x[i]=b[i];
      indiceMin=(i-nbChiffreMax+1>0?i-nbChiffreMax+1:0);
      for(j=indiceMin; j<i; j++)
		{
		    x[i]-=L[i][j]*x[j];
		}
	    x[i]/=L[i][i];
	}
}
// Resolve L U X = B
// U is upper diagonal (band of large k/2 + 1)
// L is lower diagonal (band of large k/2 + 1)  (temp de taille n)
template <class T, class Matrix1, class Matrix2, class Vector_int, class Vector1, class Vector2, class Vector3>
inline void matrix_band_resolve_LU_T(const T*,
			      const int n,const int k,
			      const Matrix1 &L,
			      const Matrix2 &U,
			      const Vector_int &t,
			      const Vector1 &b,
			      Vector2 &x,
			      Vector3 &temp)
{
      int i;
    for(i=0;i<n;i++) x[i]=b[t[i]];
    matrix_band_resolve_lower(n,k,L,x,temp);
    matrix_band_resolve_upper(n,k,U,temp,x);
}
