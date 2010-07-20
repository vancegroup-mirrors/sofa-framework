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
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/forcefield/SpringEdgeDataForceField.inl>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/simulation/tree/xml/ObjectFactory.h>
//#include <typeinfo>


namespace sofa
{

namespace component
{

namespace forcefield
{

SOFA_DECL_CLASS(SpringEdgeDataForceField)

using namespace sofa::defaulttype;


template<class DataTypes>
void create(SpringEdgeDataForceField<DataTypes>*& obj, simulation::xml::ObjectDescription* arg)
{
        simulation::xml::createWithParent< SpringEdgeDataForceField<DataTypes>, core::behavior::MechanicalState<DataTypes> >(obj, arg);
	if (obj != NULL)
	{
	    obj->parseFields( arg->getAttributeMap() );
		//if (arg->getAttribute("filename"))
		//	obj->load(arg->getAttribute("filename"));
	}
}
#ifndef SOFA_FLOAT
Creator<simulation::xml::ObjectFactory, SpringEdgeDataForceField<Vec3dTypes> > SpringEdgeDataForceFieldVec3dClass("SpringEdgeDataForceField", true);
template class SpringEdgeDataForceField<Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
Creator<simulation::xml::ObjectFactory, SpringEdgeDataForceField<Vec3fTypes> > SpringEdgeDataForceFieldVec3fClass("SpringEdgeDataForceField", true);
template class SpringEdgeDataForceField<Vec3fTypes>;
#endif
} // namespace forcefield

} // namespace component

} // namespace sofa

