/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/component/linearsolver/LapackOperations.h>

//Lapack
#include <blas3pp.h>
#include <blas2pp.h>
#include <laslv.h>
#include <lapackpp.h>

namespace sofa
{

  namespace component
  {

    namespace linearsolver
    {


//cblas_dgemm : double precision, general matrices, multiplication matrix-matrix
//              alpha*op(A)*op(B) + beta*C : with op() meaning wether or not we transpose the matrix

void applyLapackDGEMM(	double* A, bool isTransposeA,  double* B, bool isTransposeB, double* C,
					    double alpha, double beta,
						unsigned int rowsA, unsigned int columnsA, unsigned int rowsB, unsigned int columnsB)
{
	LaGenMatDouble MA(A,rowsA, columnsA,true);
	LaGenMatDouble MB(B,rowsB, columnsB,true);
	LaGenMatDouble MC(C,rowsA, columnsB,true);
	Blas_Mat_Mat_Mult(MA, MB, MC, isTransposeA, isTransposeB, alpha, beta);
}


void applyLapackDGEMM(  FullMatrix<double> &A, bool isTransposeA,  FullMatrix<double> &B, bool isTransposeB, FullMatrix<double> &C, double alpha, double beta)
{
	unsigned int rowA,colA,rowB,colB;
	if (isTransposeA) {rowA=A.colSize(); colA=A.rowSize();}
	else              {rowA=A.rowSize(); colA=A.colSize();}

	if (isTransposeB) {rowB=B.colSize(); colB=B.rowSize();}
	else              {rowB=B.rowSize(); colB=B.colSize();}


	LaGenMatDouble MA(A[0],rowA, colA,true);
	LaGenMatDouble MB(B[0],rowB, colB,true);
	LaGenMatDouble MC(C[0],rowA, colB,true);
	Blas_Mat_Mat_Mult(MA, MB, MC, isTransposeA, isTransposeB, alpha, beta);
}


//cblas_dgemv : double precision, general matrices, multiplication matrix-vector
//              alpha*op(A)*x + beta*y : with op() meaning wether or not we transpose the matrix
void applyLapackDGEMV(  double* A, bool isTransposeA,   double* x, double* y,
						double alpha, double beta,
						unsigned int rowsA, unsigned int columnsA)
{
	LaGenMatDouble MA(A,rowsA, columnsA,true);
	if (isTransposeA){
		LaVectorDouble Vx(x,rowsA);
		LaVectorDouble Vy(y,columnsA);
		Blas_Mat_Trans_Vec_Mult(MA, Vx, Vy, alpha, beta);
	}else{
		LaVectorDouble Vx(x,columnsA);
		LaVectorDouble Vy(y,rowsA);
		Blas_Mat_Vec_Mult(MA, Vx, Vy, alpha, beta);
	}
}


void applyLapackDGEMV(  FullMatrix<double> &A, bool isTransposeA,   FullVector<double> &x, FullVector<double> &y,
						double alpha, double beta)
{
	LaGenMatDouble MA(A[0],A.rowSize(),A.colSize(),true);
	LaVectorDouble Vx(x.ptr(),x.size());
	LaVectorDouble Vy(y.ptr(),y.size());
	if (isTransposeA)
		Blas_Mat_Trans_Vec_Mult(MA, Vx, Vy, alpha, beta);
	else
		Blas_Mat_Vec_Mult(MA, Vx, Vy, alpha, beta);
}


double applyLapackDDOT( const unsigned int N,  double *X,   double *Y)
{
	LaVectorDouble Vx(X,N);
	LaVectorDouble Vy(Y,N);
	return Blas_Dot_Prod(Vx, Vy);
}


double applyLapackDDOT(   FullVector<double> &X,   FullVector<double> &Y)
{
	return applyLapackDDOT(X.size(), X.ptr(), Y.ptr());
}


//Solve A.X=B
void applyLapackDGESV( double* A, double *X, double* B, int dimA,  int columnsB)
{
	LaGenMatDouble MA(A,dimA, dimA,true);
	LaGenMatDouble MB(B,dimA, columnsB,true);
	LaGenMatDouble MX(X,dimA, columnsB,true);
	LaLULinearSolve(MA, MX, MB);
}


void applyLapackDGESV( FullMatrix<double>& A, FullMatrix<double>& X, FullMatrix<double>& B)
{
	LaGenMatDouble MA(A[0],A.rowSize(), A.colSize(),true);
	LaGenMatDouble MB(B[0],A.rowSize(), B.colSize(),true);
	LaGenMatDouble MX(X[0],A.rowSize(), B.colSize(),true);
	LaLULinearSolve(MA, MX, MB);
}


void printMatrix( FullMatrix<double> &M, unsigned  int row,unsigned int col) 
{
	std::streamsize precisionSize = std::cout.precision();
	std::cout.precision(6);
	std::ios_base::fmtflags flagPrecision = std::cout.setf(std::ios::fixed);


	for (unsigned int r=0;r<row;r++)
	{
		if (r!=0) std::cout << "|\n|";
		else      std::cout << "|";

		for (unsigned int c=0;c<col;++c)
		{
			std::cout << M.element(r,c) << "\t";
		}
	}

	std::cout << "|\n";
	std::cout.precision(precisionSize);
	std::cout.setf(flagPrecision);
}


void printMatrix( FullMatrix<double> &M) 
{
	printMatrix(M,M.rowSize(),M.colSize());
}


void printVector( FullVector<double> &V, unsigned  int row) 
{
	std::streamsize precisionSize = std::cout.precision();
	std::cout.precision(6);
	std::ios_base::fmtflags flagPrecision = std::cout.setf(std::ios::fixed);


	for (unsigned int r=0;r<row;r++)
	{
		if (r!=0) std::cout << "|\n|";
		else      std::cout << "|";

		std::cout << V.element(r) << "\t";
	}

	std::cout << "|\n";
	std::cout.precision(precisionSize);
	std::cout.setf(flagPrecision);
}


void printVector( FullVector<double> &V) 
{
	printVector(V, V.size());
}



    }
  }
}
