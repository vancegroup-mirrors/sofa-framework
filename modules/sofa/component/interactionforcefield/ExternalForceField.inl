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
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_EXTERNALFORCEFIELD_INL
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_EXTERNALFORCEFIELD_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/componentmodel/behavior/ForceField.inl>
#include <sofa/component/interactionforcefield/ExternalForceField.h>
#include <sofa/component/topology/MeshTopology.h>
#include <sofa/component/topology/GridTopology.h>
#include <sofa/helper/PolarDecompose.h>
#include <sofa/helper/gl/template.h>
#include <assert.h>
#include <iostream>
#if defined (__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
using std::cerr;
using std::endl;


namespace sofa
{

namespace component
{

namespace interactionforcefield
{

using std::cerr;
using std::endl;

using namespace sofa::defaulttype;


template<class DataTypes>
ExternalForceField<DataTypes>::ExternalForceField ()
: m_forces(dataField(&m_forces,"forces","Values of the forces"))
, m_indices(dataField(&m_indices,"indices","Indices of the particles undergoing the forces"))
{
}

template<class DataTypes>
void ExternalForceField<DataTypes>::addForce (VecDeriv& f, const VecCoord&/* p*/, const VecDeriv& /*v*/)
{
    if( this->f_printLog.getValue() )
        cerr<<"ExternalForceField<DataTypes>::addForce"<<endl;
    const helper::vector<unsigned>& indices = m_indices.getValue();
    const VecDeriv& forces = m_forces.getValue();
    for( unsigned i=0; i<indices.size(); ++i )
    {
        std::cout << "f["<<i<<"]+="<<forces[i]<<std::endl;
        f[indices[i]] += forces[i];
    }
}

template <class DataTypes> 
        double ExternalForceField<DataTypes>::getPotentialEnergy(const VecCoord&)
{
    cerr<<"ExternalForceField::getPotentialEnergy-not-implemented !!!"<<endl;
    return 0;
}

} // namespace interactionforcefield

} // namespace component

} // namespace sofa

#endif
