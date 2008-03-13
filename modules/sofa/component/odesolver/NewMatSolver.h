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
#ifndef SOFA_COMPONENT_ODESOLVER_NEWMATSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_NEWMATSOLVER_H

#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/simulation/tree/OdeSolverImpl.h>
#include <sofa/defaulttype/NewMatMatrix.h>
#include <sofa/defaulttype/NewMatVector.h>

namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;

class NewMatSolver : public sofa::simulation::tree::OdeSolverImpl
{
public:

	NewMatSolver();
	virtual ~NewMatSolver();
    void solve(double dt);

private:
	bool updateMatrix;
	unsigned int nbRow, nbCol;
	NewMatMatrix *globalStiffnessMatrix;
	NewMatVector *externalForcesGlobalVector, *resultGlobalVector;
};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif

