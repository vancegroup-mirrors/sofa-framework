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
// Author: Pierre-Jean Bensoussan, INRIA-LIFL, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/odesolver/MKLSolver.h>
#include <sofa/core/ObjectFactory.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sofa/helper/system/thread/CTime.h>


using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using namespace core::componentmodel::behavior;
using namespace helper::system::thread;


MKLSolver::MKLSolver()
{
    globalStiffnessMatrix = new MKLMatrix();
	externalForcesGlobalVector = new MKLVector();
    
	updateMatrix = true;

    nbRow = 0;
    nbCol = 0;
}

MKLSolver::~MKLSolver()
{
    delete globalStiffnessMatrix;
	delete externalForcesGlobalVector;
}

void MKLSolver::solve(double dt)
{
    OdeSolver* group = this;
    MultiVector f(group, VecId::force());
    MultiVector pos(group, VecId::position());
	MultiVector dx(group, VecId::null());
    
    bool printLog = f_printLog.getValue();

    if( printLog )
    {
        cerr<<"MKLSolver, dt = "<< dt <<endl;
        cerr<<"MKLSolver, initial x = "<< pos <<endl;
    }

    if (this->updateMatrix)
    {
		ctime_t t0;
		if (printLog)
			t0 = CTime::getTime();

		getMatrixDimension(&nbRow, &nbCol);
        globalStiffnessMatrix->resize(nbRow, nbCol);
		externalForcesGlobalVector->resize(nbRow);

		// Compute and assemble the global "Mechanical" Matrix threw forcefield, masses...
		// Then applies baseConstraints such as fixed point constraints
		addMBK_ToMatrix(globalStiffnessMatrix,0,0,1);

		// Compute and assemble the global "Mechanical" Vector threw forcefield, masses...
		// Then applies baseConstraints such as fixed point constraints 
		addMBKdx_ToVector(externalForcesGlobalVector,dx,1,0,0);
		
		// Solve dx = Fext / K
        globalStiffnessMatrix->solve(externalForcesGlobalVector);

		// Update position using the solver result
		multiVectorPeqBaseVector(pos, externalForcesGlobalVector);

		// This solver is static and should be solved only one time
        updateMatrix = false;

		if (printLog)
		{
			ctime_t t1 = CTime::getTime() - t0;
			std::cout << "Integration has taken " << t1 / CTime::getRefTicksPerSec() << std::endl;
		}
    }
}

SOFA_DECL_CLASS(MKLSolver)

int MKLSolverClass = core::RegisterObject("Solver using MKL Matrix Library")
.add< MKLSolver >();

} // namespace odesolver

} // namespace component

} // namespace sofa

