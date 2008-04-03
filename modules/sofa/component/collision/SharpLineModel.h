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
#ifndef SOFA_COMPONENT_COLLISION_SHARPLINEMODEL_H
#define SOFA_COMPONENT_COLLISION_SHARPLINEMODEL_H

#include <sofa/component/collision/LineModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace sofa::component::topology;

class SharpLineModel : public LineModel
{
protected:
    typedef Vector3::value_type Real;
    struct SharpLineData
    {
        Vector3 normal;
        Real cosAngle;
        SharpLineData(const Vector3& n = Vector3(), Real c = (Real)0.5f)
        : normal(n), cosAngle(c)
        {
        }
    };

    sofa::helper::vector<SharpLineData> selems;

public:
    Data<Real> minAngle;
    SharpLineModel();

    Vector3 getNormal(int index);
    Real getCosAngle(int index);

    virtual void init();
    virtual void resize(int size);
    virtual void draw(int index);
    virtual void draw();
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
