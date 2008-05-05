/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include "CudaLCP.h"
#include "CudaTypes.h" 

namespace sofa
{

namespace gpu
{

namespace cuda
{

extern "C"
{
	void CudaLCP_MultVector(int dim,int index, const void * m,const void * f,void * r);
	int CudaLCP_MultVector_ResSize(unsigned int dim);
	void CudaLCP_ComputeError(int compteur2,int sizeTmp, const void * tmp, const void * m,const void * q,void * f,void * res,void * error);
}

//using namespace sofa::gpu;
/* Resoud un LCP écrit sous la forme U = q + M.F
 * dim : dimension du pb
 * res[0..dim-1] = U
 * res[dim..2*dim-1] = F
 */
static sofa::gpu::cuda::CudaMatrix<float> cuda_M;
static sofa::gpu::cuda::CudaVector<float> cuda_res;
static sofa::gpu::cuda::CudaVector<float> cuda_f;
static sofa::gpu::cuda::CudaVector<float> cuda_tmp;
static sofa::gpu::cuda::CudaVector<float> cuda_q;	
static sofa::gpu::cuda::CudaVector<float> cuda_err;		
	
void readVector(CudaVector<float>& v,double * array,int dim) {
	v.resize(dim);
	for (int i=0;i<dim;i++) {
		v[i] = (float) array[i];
	}
}
	
void readMatrix(CudaMatrix<float>& m,double ** array,int dim) {
	m.resize(dim,dim);
	for (int i=0;i<dim;i++) {
		for (int j=0;j<dim;j++) {
			m[i][j] = (float) array[i][j];
		}
	}
}	

void CudaLCP::CudaGaussSeidelLCP1(int dim, double * q, double ** M, double * res, double &tol, int &numItMax) {
	int compteur;	// compteur de boucle
	int compteur2;	// compteur de boucle

	double error=0.0;
	
	std::cout << "CudaLCP dim="<<dim<<std::endl;
	
	//on charge les vecteurs sur GPU
	readVector(cuda_f,res+dim,dim);
	readVector(cuda_res,res,dim);
	readVector(cuda_q,q,dim);
	readMatrix(cuda_M,M,dim);
		
	//On prend un vecteur temporaire pour stocker les reusltats des != multiplications
	int tmpsize = CudaLCP_MultVector_ResSize(dim);
	cuda_tmp.resize(tmpsize);
	
	//On transmet un vecteur de dimention 1 pour stocker l'erreur
	cuda_err.resize(1);
		
	for (compteur=0; compteur<numItMax; compteur++) {
		cuda_err[0] = 0.0;
		for (compteur2=0; compteur2<dim; compteur2++) {
			//int indM = dim*compteur2+compteur2;
			CudaLCP_MultVector(dim,compteur2,cuda_M.deviceRead(0,compteur2),cuda_f.deviceRead(),cuda_tmp.deviceWrite());
			CudaLCP_ComputeError(compteur2,tmpsize,cuda_tmp.deviceRead(),cuda_M.deviceRead(compteur2,compteur2),cuda_q.deviceRead(),cuda_f.deviceWrite(),cuda_res.deviceWrite(),cuda_err.deviceWrite());
		}

		if (cuda_err[0] < tol) break;
	}
	
	error = cuda_err[0];

	for (compteur=0; compteur<dim; compteur++) res[compteur] = cuda_f[compteur];

	if (error >= tol) {
		std::cout << "No convergence in gaussSeidelLCP1 : error = " << error << std::endl;
		//	afficheLCP(q, M, res, dim);
	}
}

} // namespace cuda

} // namespace gpu

} // namespace sofa

/*
			float r = q[compteur2];
			for (int i=0;i<tmpsize;++i)
				r += cuda_tmp[i];
			
			//float r = q[compteur2];
			//for (int i=0;i<dim;++i)
			//	if (i!=compteur2)
			//		r += cuda_Mi[compteur2*dim+i]*cuda_f[i];
			
			cuda_res[compteur2] = r;
			//cuda_res[compteur2] -= M[compteur2][compteur2]* cuda_res[dim+compteur2];
			f_1 = cuda_f[compteur2];

			if (r<0) cuda_f[compteur2]=-r/M[compteur2][compteur2];
			else cuda_f[compteur2]=(double)0.0;
			
			error +=fabs( M[compteur2][compteur2] * (cuda_f[compteur2] - f_1) );
*/
