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
#ifndef SOFA_COMPONENT_ENGINE_TRIANGLESINSPHEREROI_INL
#define SOFA_COMPONENT_ENGINE_TRIANGLESINSPHEREROI_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/component/engine/TrianglesInSphereROI.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>

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
TrianglesInSphereROI<DataTypes>::TrianglesInSphereROI()
: isVisible( initData (&isVisible, bool (true), "isVisible", "is Visible ?") )
, centers( initData(&centers, "centers", "Center(s) of the sphere(s)") )
, radii( initData(&radii, "radii", "Radius(i) of the sphere(s)") )
, normal( initData(&normal, "normal", "Normal direction of the triangles (if angle > 0)") )
, angle( initData(&angle, (Real)0, "angle", "Max angle between the normal of the selected triangle and the specified normal direction") )
, f_X0( initData (&f_X0, "rest_position", "Rest position coordinates of the degrees of freedom") )
, f_triangles(initData(&f_triangles, "triangles", "List of triangle indices"))
, f_indices( initData(&f_indices,"indices","Indices of the triangles contained in the ROI") )
, f_pointIndices( initData(&f_pointIndices,"pointIndices","Indices of the points of the triangles contained in the ROI") )
, _drawSize( initData(&_drawSize,0.0,"drawSize","0 -> point based rendering") )
{
    f_indices.beginEdit()->push_back(0);
    f_indices.endEdit();
}

template <class DataTypes>
void TrianglesInSphereROI<DataTypes>::init()
{
    if (f_X0.getValue().empty())
    {
	MechanicalState<DataTypes>* mstate;
	this->getContext()->get(mstate);
	if (mstate)
	{
	    BaseData* parent = mstate->findField("rest_position");
	    if (parent)
	    {
		f_X0.setParent(parent);
		f_X0.setReadOnly(true);
	    }
	}
    }
    if (f_triangles.getValue().empty())
    {
        BaseMeshTopology* topology = dynamic_cast<BaseMeshTopology*>(getContext()->getTopology());
        if (topology != NULL)
        {
            BaseData* parent = topology->findField("triangles");
            if (parent != NULL)
            {
                f_triangles.setParent(parent);
                f_triangles.setReadOnly(true);
            }
            else
            {
                sout << "ERROR: Topology " << topology->getName() << " does not contain triangles" << sendl;
            }
        }
        else
        {
            sout << "ERROR: Topology not found. Triangles in sphere can not be computed" << sendl;
        }
    }

    addInput(&f_X0);
    addInput(&f_triangles);
    addInput(&centers);
    addInput(&radii);
    addInput(&normal);
    addInput(&angle);

    addOutput(&f_indices);
    addOutput(&f_pointIndices);

    setDirtyValue();
}

template <class DataTypes>
void TrianglesInSphereROI<DataTypes>::reinit()
{
    update();
}

template <class DataTypes>
bool TrianglesInSphereROI<DataTypes>::containsTriangle(const Vec3& c, const Real& r, const BaseMeshTopology::Triangle& triangle)
{
    for (unsigned int i=0; i<3; ++i)
    {
        Coord p = (*x0)[triangle[i]];

        if((p-c).norm() > r)
            return false;
    }
    return true;
}

template <class DataTypes>
void TrianglesInSphereROI<DataTypes>::update()
{
    cleanDirty();

    const helper::vector<Vec3>& c = (centers.getValue());
    const helper::vector<Real>& r = (radii.getValue());
    Real a = angle.getValue();
    Coord norm = normal.getValue();
    if (a>0)
        norm.normalize();

    SetTriangle& indices = *(f_indices.beginEdit());
    SetIndex& pointIndices = *(f_pointIndices.beginEdit());
    
    indices.clear();
    pointIndices.clear();

    x0 = &f_X0.getValue();

    const BaseMeshTopology::SeqTriangles* triangles = &f_triangles.getValue();

    if (c.size() == r.size())
    {
        for(unsigned int i=0; i<triangles->size(); ++i)
        {
            const BaseMeshTopology::Triangle& triangle = (*triangles)[i];
            bool inside = false;
            for (unsigned int j=0;j<c.size();++j)
                if (containsTriangle(c[j], r[j], triangle)) inside = true;
            if (inside)
            {
                if (a > 0)
                {
                    Coord n = cross((*x0)[triangle[2]]-(*x0)[triangle[0]], (*x0)[triangle[1]]-(*x0)[triangle[0]]);
                    n.normalize();
                    if (dot(n,norm) < cos(a*M_PI/180.0)) continue;
                }

                indices.push_back(i);
                pointIndices.push_back((*triangles)[i][0]);
                pointIndices.push_back((*triangles)[i][1]);
                pointIndices.push_back((*triangles)[i][2]);
            }
        }
    }

    f_indices.endEdit();
    f_pointIndices.endEdit();
}

template <class DataTypes>
void TrianglesInSphereROI<DataTypes>::draw()
{
    if (!this->getContext()->getShowBehaviorModels() || !isVisible.getValue())
        return;

    if( _drawSize.getValue() == 0) // old classical drawing by points
    {
        ///draw the boxes
        const helper::vector<Vec3>& c=centers.getValue();
        const helper::vector<Real>& r=radii.getValue();

        for (unsigned int i=0;i<c.size() && i<r.size();++i)
        {
        	helper::gl::drawWireSphere(c[i], (float)(r[i]/2.0));

                if (angle.getValue() > 0)
                {
		  helper::gl::drawCone(c[i], c[i] + normal.getValue()*(cos(angle.getValue()*M_PI/180.0)*r[i]), 0, (float)sin(angle.getValue()*M_PI/180.0)*((float)r[i]));
                }
        }
    }
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
