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
#ifndef SOFA_CORE_OBJECTMODEL_CUDAROTATIONMATRIX_H
#define SOFA_CORE_OBJECTMODEL_CUDAROTATIONMATRIX_H

#include <sofa/gpu/cuda/CudaTypes.h>
#include <sofa/gpu/cuda/CudaTypesBase.h>
#include <sofa/gpu/cuda/CudaMatrixUtils.h>

namespace sofa {

namespace gpu {

namespace cuda {

template<class Real>
class CudaRotationMatrix : public defaulttype::BaseMatrix {
  public:    
    virtual unsigned int rowSize(void) const {
	return data.size()/3;
    }
    
    /// Number of columns
    virtual unsigned int colSize(void) const {
	return data.size()/3;
    }
    
    /// Read the value of the element at row i, column j (using 0-based indices)
    virtual SReal element(int i, int j) const {
	int bd = (i/3)*3;
	if ((j<bd) || (j>bd+2)) return 0.0 ;
      
	return data[i*3+j-bd];
    }
    
    /// Resize the matrix and reset all values to 0
    virtual void resize(int nbRow, int nbCol) {
	if (nbRow!=nbCol) return;
	data.resize(nbRow*3);
    }

    /// Reset all values to 0
    virtual void clear() {
	for (unsigned i=0;i<data.size();i++) data[i] = 0.0;
    }
    
    /// Write the value of the element at row i, column j (using 0-based indices)
    virtual void set(int i, int j, double v) {
	int bd = (i/3)*3;
	if ((j<bd) || (j>bd+2)) return;
	data[i*3+j-bd] = v;      
    }
    
    /// Add v to the existing value of the element at row i, column j (using 0-based indices)
    virtual void add(int i, int j, double v) {
	int bd = (i/3)*3;
	if ((j<bd) || (j>bd+2)) return;
	
	data[i*3+j-bd] += v;
    }
    
    virtual CudaVector<Real> & getCudaVector() {
	return data;
    }

    virtual void opMulV(defaulttype::BaseVector* result, const defaulttype::BaseVector* v) const {
	//Solve lv = R * lvR
	if (CudaBaseVector<Real> * rvR = dynamic_cast<CudaBaseVector<Real> * >(result)) {
	  if (const CudaBaseVector<Real> * rv = dynamic_cast<const CudaBaseVector<Real> * >(v)) {
		CudaMatrixUtilsKernels<Real>::trimatrix_vector_product(3*rvR->size(),
									data.deviceRead(),
									rv->getCudaVector().deviceRead(),
									rvR->getCudaVector().deviceWrite());
		return;
	  }
	} 
	
	unsigned int k = 0,l = 0;
	while (k < data.size()) {
		result->set(l+0,data[k + 0] * v->element(l+0) + data[k + 1] * v->element(l+1) + data[k + 2] * v->element(l+2));
		result->set(l+1,data[k + 3] * v->element(l+0) + data[k + 4] * v->element(l+1) + data[k + 5] * v->element(l+2));
		result->set(l+2,data[k + 6] * v->element(l+0) + data[k + 7] * v->element(l+1) + data[k + 8] * v->element(l+2));
		l+=3;
		k+=9;
	}
    }
    
    virtual void opMulTV(defaulttype::BaseVector* result, const defaulttype::BaseVector* v) const {
	//Solve lv = R * lvR
	if (CudaBaseVector<Real> * rvR = dynamic_cast<CudaBaseVector<Real> * >(result)) {
	  if (const CudaBaseVector<Real> * rv = dynamic_cast<const CudaBaseVector<Real> * >(v)) {
		CudaMatrixUtilsKernels<Real>::trimatrixtr_vector_product(3*rvR->size(),
									 data.deviceRead(),
									 rv->getCudaVector().deviceRead(),
									 rvR->getCudaVector().deviceWrite());
		return;
	  }
	}       
      
      	unsigned int k = 0,l = 0;
	while (k < data.size()) {
		result->set(l+0,data[k + 0] * v->element(l+0) + data[k + 3] * v->element(l+1) + data[k + 6] * v->element(l+2));
		result->set(l+1,data[k + 1] * v->element(l+0) + data[k + 4] * v->element(l+1) + data[k + 7] * v->element(l+2));
		result->set(l+2,data[k + 2] * v->element(l+0) + data[k + 5] * v->element(l+1) + data[k + 8] * v->element(l+2));
		l+=3;
		k+=9;
	}
    }
   
    /// multiply the transpose current matrix by m matrix and strore the result in m
    virtual void opMulTM(defaulttype::BaseMatrix * bresult,defaulttype::BaseMatrix * bm) {
	if (CudaRotationMatrix<Real> * m = dynamic_cast<CudaRotationMatrix<Real> * >(bm)) {
	  if (CudaRotationMatrix<Real> * result = dynamic_cast<CudaRotationMatrix<Real> * >(bresult)) {
	    CudaMatrixUtilsKernels<Real>::trimatrix_trimatrixtr_product(data.size(),
									m->data.deviceRead(),
									data.deviceRead(),
									result->data.deviceWrite());
	    return;
	  }
	}
	defaulttype::BaseMatrix::opMulTM(bresult,bm);
    }
   
private :
    CudaVector<Real> data;
};


} // namespace misc

} // namespace component

} // namespace sofa

#endif
