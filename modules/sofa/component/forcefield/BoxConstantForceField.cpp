/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/component/forcefield/BoxConstantForceField.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

  namespace component
  {

    namespace forcefield
    {
			
			
      using namespace sofa::defaulttype;


      SOFA_DECL_CLASS(BoxConstantForceField)

      int BoxConstantForceFieldClass = core::RegisterObject("Constant forces applied to degrees of freedom contained in the bbox")
#ifndef SOFA_FLOAT
	.add< BoxConstantForceField<Vec3dTypes> >()
	.add< BoxConstantForceField<Vec2dTypes> >()
	.add< BoxConstantForceField<Vec1dTypes> >()
	.add< BoxConstantForceField<Vec6dTypes> >()
	.add< BoxConstantForceField<Rigid3dTypes> >()
	.add< BoxConstantForceField<Rigid2dTypes> >()
#endif
#ifndef SOFA_DOUBLE
	.add< BoxConstantForceField<Vec3fTypes> >()
	.add< BoxConstantForceField<Vec2fTypes> >()
	.add< BoxConstantForceField<Vec1fTypes> >()
	.add< BoxConstantForceField<Vec6fTypes> >()
	.add< BoxConstantForceField<Rigid3fTypes> >()
	.add< BoxConstantForceField<Rigid2fTypes> >()
#endif
	;
#ifndef SOFA_FLOAT
      template class BoxConstantForceField<Vec3dTypes>;
      template class BoxConstantForceField<Vec2dTypes>;
      template class BoxConstantForceField<Vec1dTypes>;
      template class BoxConstantForceField<Vec6dTypes>;
      template class BoxConstantForceField<Rigid3dTypes>;
      template class BoxConstantForceField<Rigid2dTypes>;
#endif
#ifndef SOFA_DOUBLE
      template class BoxConstantForceField<Vec3fTypes>;
      template class BoxConstantForceField<Vec2fTypes>;
      template class BoxConstantForceField<Vec1fTypes>;
      template class BoxConstantForceField<Vec6fTypes>;
      template class BoxConstantForceField<Rigid3fTypes>;
      template class BoxConstantForceField<Rigid2fTypes>;
#endif
    }	
  }
}



