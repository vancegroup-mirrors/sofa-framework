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
#ifndef SOFA_COMPONENT_LINEARSOLVER_LAPACKOPERATION_H
#define SOFA_COMPONENT_LINEARSOLVER_LAPACKOPERATION_H

#include <sofa/component/component.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/component/linearsolver/FullVector.h>
#include <vector>
#include <fstream>
#include <iostream>


namespace sofa
{

  namespace component
  {

    namespace linearsolver
    {


       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGEMM( double* A, bool isTransposeA,  double* B, bool isTransposeB, double* C,
				    double alpha, double beta,
				    unsigned int rowsA, unsigned int columnsA, unsigned int rowsB, unsigned int columnsB);

       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGEMM(  FullMatrix<double> &A, bool isTransposeA,  FullMatrix<double> &B, bool isTransposeB, FullMatrix<double> &C, double alpha, double beta);

      //cblas_dgemv : double precision, general matrices, multiplication matrix-vector
      //              alpha*op(A)*x + beta*y : with op() meaning wether or not we transpose the matrix

       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGEMV(  double* A, bool isTransposeA,   double* x, double* y,
				     double alpha, double beta,
			      unsigned int rowsA, unsigned int columnsA);

       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGEMV(  FullMatrix<double> &A, bool isTransposeA,   FullVector<double> &x, FullVector<double> &y,
			      double alpha, double beta);

       double SOFA_COMPONENT_LINEARSOLVER_API applyLapackDDOT( const unsigned int N,  double *X,   double *Y);

       double SOFA_COMPONENT_LINEARSOLVER_API applyLapackDDOT(   FullVector<double> &X,   FullVector<double> &Y);

      //Solve A.X=B
       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGESV( double* A, double*X, double* B, int dimA,  int columnsB);
       void SOFA_COMPONENT_LINEARSOLVER_API applyLapackDGESV( FullMatrix<double>& A, FullMatrix<double>& X, FullMatrix<double>& B);

       void SOFA_COMPONENT_LINEARSOLVER_API printMatrix( FullMatrix<double>& M, unsigned  int row,unsigned int col) ; 
       void SOFA_COMPONENT_LINEARSOLVER_API printMatrix( FullMatrix<double>& M) ;  
       void SOFA_COMPONENT_LINEARSOLVER_API printVector( FullVector<double>& V, unsigned  int row,unsigned int col) ; 
       void SOFA_COMPONENT_LINEARSOLVER_API printVector( FullVector<double>& V) ;  

    } // namespace linearsolver

  } // namespace component

} // namespace sofa

#endif
