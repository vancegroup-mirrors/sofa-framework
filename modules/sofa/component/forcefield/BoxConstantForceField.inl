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
#ifndef SOFA_COMPONENT_BOXCONSTANTFORCEFIELD_INL
#define SOFA_COMPONENT_BOXCONSTANTFORCEFIELD_INL


#include <sofa/component/forcefield/BoxConstantForceField.h>

namespace sofa
{

	namespace component
	{

		namespace forcefield
		{
			
			using namespace sofa::helper;
			
			
			template<class DataTypes>
					BoxConstantForceField<DataTypes>::BoxConstantForceField()
				: _box( initData( &_box, Vec6(0,0,0,1,1,1), "box", "DOFs in the box defined by xmin,ymin,zmin, xmax,ymax,zmax undergo a constant force") )
					, _force( initData( &_force, Deriv(), "force", "the constant force to apply"))
					{
					}
			
			template<class DataTypes>
					void BoxConstantForceField<DataTypes>::init()
					{
						ConstantForceField<DataTypes>::init();
						vector <unsigned> indices;
						const Vec6& b=_box.getValue();
						this->mstate->getIndicesInSpace( indices, b[0],b[3],b[1],b[4],b[2],b[5] );
						for(int i = 0; i < (int)indices.size(); i++)
						{
							this->points.beginEdit()->push_back(indices[i]);
							this->forces.beginEdit()->push_back(_force.getValue());
						}
					}
		}	
	}
}


#endif

