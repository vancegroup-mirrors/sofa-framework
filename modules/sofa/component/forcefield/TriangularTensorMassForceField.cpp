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
#include <sofa/component/forcefield/TriangularTensorMassForceField.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/topology/MeshTopology.h>
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <vector>
#include <sofa/defaulttype/Vec3Types.h>



// #define DEBUG_TRIANGLEFEM

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

using std::cerr;
using std::cout;
using std::endl;

SOFA_DECL_CLASS(TriangularTensorMassForceField)

using namespace sofa::defaulttype;

template class TriangularTensorMassForceField<Vec3dTypes>;
template class TriangularTensorMassForceField<Vec3fTypes>;


// Register in the Factory
int TriangularTensorMassForceFieldClass = core::RegisterObject("Linear Elastic Membrane on a Triangular Mesh")
.add< TriangularTensorMassForceField<Vec3dTypes> >()
.add< TriangularTensorMassForceField<Vec3fTypes> >()
;



} // namespace forcefield

} // namespace Components

} // namespace Sofa

