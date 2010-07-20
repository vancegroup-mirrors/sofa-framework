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
#ifndef SOFA_CORE_OBJECTMODEL_BASEROTATIONFINDER_H
#define SOFA_CORE_OBJECTMODEL_BASEROTATIONFINDER_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/vector.h>

namespace sofa {

namespace component {

namespace misc {

/// Direct linear solver based on Sparse LDL^T factorization, implemented with the CSPARSE library
class BaseRotationFinder : public virtual sofa::core::objectmodel::BaseObject {
  public:
      ///for a block-diagonal matrix like :
      ///a b c 0 0 0 0 0 0 
      ///d e f 0 0 0 0 0 0
      ///g h i 0 0 0 0 0 0 
      ///0 0 0 j k l 0 0 0 
      ///0 0 0 m n o 0 0 0
      ///0 0 0 p q r 0 0 0
      ///0 0 0 0 0 0 s t u
      ///0 0 0 0 0 0 v w x
      ///0 0 0 0 0 0 y z ù
      ///write a vector like :
      /// a b c d e f g h i j k l m n o p ...
      
      virtual void getRotations(defaulttype::BaseVector * m) = 0;
};

} // namespace misc

} // namespace component

} // namespace sofa

#endif
