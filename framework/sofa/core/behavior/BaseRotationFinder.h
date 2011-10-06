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
#ifndef SOFA_COMPONENT_MISC_BASEROTATIONFINDER_H
#define SOFA_COMPONENT_MISC_BASEROTATIONFINDER_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/vector.h>

namespace sofa
{

namespace component
{

namespace misc
{

/// Direct linear solver based on Sparse LDL^T factorization, implemented with the CSPARSE library
class BaseRotationFinder : public virtual sofa::core::objectmodel::BaseObject
{
public:

    virtual void getRotations(defaulttype::BaseMatrix * m,int offset = 0) = 0;
};

} // namespace misc

} // namespace component

} // namespace sofa

#endif