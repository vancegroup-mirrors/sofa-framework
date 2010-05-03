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
#ifndef SOFA_COMPONENT_LINEARSOLVER_SparsePARDISOSolver_H
#define SOFA_COMPONENT_LINEARSOLVER_SparsePARDISOSolver_H

#include <sofa/core/componentmodel/behavior/LinearSolver.h>
#include <sofa/component/linearsolver/MatrixLinearSolver.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/component/linearsolver/SparseMatrix.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/helper/map.h>
#include <math.h>

// include all headers included in taucs.h to fix errors on macx
#include <complex.h>
#include <assert.h>
#include <float.h>
#include <stdlib.h>

namespace sofa {

namespace component {

namespace linearsolver {

/// Linear system solver using the conjugate gradient iterative algorithm
template<class TMatrix, class TVector>
class SparsePARDISOSolver : public sofa::component::linearsolver::MatrixLinearSolver<TMatrix,TVector>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(SparsePARDISOSolver,TMatrix,TVector),SOFA_TEMPLATE2(sofa::component::linearsolver::MatrixLinearSolver,TMatrix,TVector));

    typedef TMatrix Matrix;
    typedef TVector Vector;
    typedef typename Matrix::Real Real;
    typedef sofa::component::linearsolver::MatrixLinearSolver<TMatrix,TVector> Inherit;
    typedef sofa::core::componentmodel::behavior::BaseMechanicalState::VecId VecId;

    //Data< helper::vector<std::string> > f_options;
    Data<int> f_symmetric;

    Data<bool> f_verbose;
    Data<std::map < std::string, sofa::helper::vector<double> > > f_graph;

    SparsePARDISOSolver();
    ~SparsePARDISOSolver();
    virtual void init();

    void solve (Matrix& M, Vector& x, Vector& b);
    void invert(Matrix& M);

protected:
    Matrix Mfiltered;

    void*  pardiso_pt[64];
    int    pardiso_iparm[64];
    double pardiso_dparm[64];
    int pardiso_initerr;
    int pardiso_mtype;
    int callPardiso(int phase, Vector* vx = NULL, Vector* vb = NULL);
    bool factorized;
};

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
