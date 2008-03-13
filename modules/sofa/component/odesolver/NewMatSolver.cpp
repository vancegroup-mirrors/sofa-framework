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
#include <sofa/component/odesolver/NewMatSolver.h>
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

NewMatSolver::NewMatSolver()
{
    globalStiffnessMatrix = new NewMatMatrix();
	externalForcesGlobalVector = new NewMatVector();
	resultGlobalVector = new NewMatVector();
    
	updateMatrix = true;

    nbRow = 0;
    nbCol = 0;
}

NewMatSolver::~NewMatSolver()
{
	delete globalStiffnessMatrix;
	delete externalForcesGlobalVector;
	delete resultGlobalVector;
}

void NewMatSolver::solve(double dt)
{
    OdeSolver* group = this;
    MultiVector f(group, VecId::force());
    MultiVector pos(group, VecId::position());
	MultiVector dx(group, VecId::null());
    
    bool printLog = f_printLog.getValue();

    if( printLog )
    {
        cerr<<"NewMatSolver, dt = "<< dt <<endl;
        cerr<<"NewMatSolver, initial x = "<< pos <<endl;
    }

    if (this->updateMatrix)
    {
		
		ctime_t t0 = CTime::getTime();

		getMatrixDimension(&nbRow, &nbCol);
        globalStiffnessMatrix->resize(nbRow, nbCol);
		externalForcesGlobalVector->resize(nbRow);
		resultGlobalVector->resize(nbRow);

		// Compute and assemble the global "Mechanical" Matrix threw forcefield, masses...
		// Then applies baseConstraints such as fixed point constraints
		addMBK_ToMatrix(globalStiffnessMatrix,0,0,1);

		// Compute and assemble the global "Mechanical" Vector threw forcefield, masses...
		// Then applies baseConstraints such as fixed point constraints 
		addMBKdx_ToVector(externalForcesGlobalVector,dx,1,0,0);
		
		// Solve dx = Fext / K
        globalStiffnessMatrix->solve(externalForcesGlobalVector, resultGlobalVector);

		// Update position using the solver result
		multiVectorPeqBaseVector(pos, resultGlobalVector);

		// This solver is static and should be solved only one time
        updateMatrix = false;

//		FILE *f = fopen("debugMAT.txt","w+");
//		for (unsigned int i=0; i<nbRow; i++)
//		{
//			for (unsigned int j=0; j<nbCol; j++)
//				fprintf(f,"%.0f\t",mat->element(i, j));
//			fprintf(f,"\n");
//		}
//		fclose(f);

		ctime_t t1 = CTime::getTime() - t0;
		std::cout << "Integration has taken " << t1 / CTime::getRefTicksPerSec() << std::endl;
    }
}

SOFA_DECL_CLASS(NewMatSolver)

    int NewMatSolverClass = core::RegisterObject("Solver using a NewMat Matrix Library")
    .add< NewMatSolver >();

} // namespace odesolver

} // namespace component

} // namespace sofa

