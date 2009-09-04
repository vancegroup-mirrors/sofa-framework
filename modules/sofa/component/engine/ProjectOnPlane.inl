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
#ifndef SOFA_COMPONENT_ENGINE_PROJECTONPLANE_INL
#define SOFA_COMPONENT_ENGINE_PROJECTONPLANE_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/component/engine/ProjectOnPlane.h>
#include <sofa/helper/gl/template.h>
#include <math.h>
namespace sofa
{

namespace component
{

namespace engine
{

using namespace sofa::helper;
using namespace sofa::defaulttype;
using namespace core::objectmodel;

template <class DataTypes>
ProjectOnPlane<DataTypes>::ProjectOnPlane()
: originPtr( initDataPtr(&originPtr,&origin, "origin", "a 3d point on the plane") )
, f_inputX( initData (&f_inputX, "input_position", "input array of 3d points") )
, f_outputX( initData (&f_outputX, "output_position", "output array of 3d points projected on a plane") )
, normalPtr(initDataPtr(&normalPtr,&normal, "normal", "plane normal"))
{
}

template <class DataTypes>
void ProjectOnPlane<DataTypes>::init()
{
   /* if (!f_outputX.isSet())
    {
        BaseData* parent = mstate->findField("position");
        f_outputX.setParentValue(parent);
        parent->addOutput(&f_outputX);
        f_outputX.setReadOnly(true);
    } */

    addInput(&f_inputX);
    addOutput(&f_outputX);

    setDirty();
	/// check if the normal is of norm 1
	if (fabs((normal.norm2()-1.0))>1e-10) {
		normal/=normal.norm();
	}
}

template <class DataTypes>
void ProjectOnPlane<DataTypes>::reinit()
{
    update();
}



template <class DataTypes>
void ProjectOnPlane<DataTypes>::update()
{
    dirty = false;


    const helper::vector<Coord>& in = f_inputX.getValue();
    helper::vector<Coord>& out = *(f_outputX.beginEdit());

	out.resize(in.size());

    for (unsigned int i=0;i< in.size(); ++i)
    {
		out[i]=in[i]+normal*dot((origin-in[i]),normal);
    }

//   f_inputX.endEdit();
   f_outputX.endEdit();

}

template <class DataTypes>
void ProjectOnPlane<DataTypes>::draw()
{

}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
