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
#include <sofa/component/collision/SharpLineModel.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/component/collision/Line.h>
#include <sofa/core/CollisionElement.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/rmath.h>
#include <vector>
#include <sofa/helper/system/gl.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(SharpLineModel)

int SharpLineModelClass = core::RegisterObject("Collision model representing a set of sharp lines")
.add< SharpLineModel >()
.addAlias("SharpLine")
;

SharpLineModel::SharpLineModel()
: minAngle( initData( &minAngle, (Real)91.0, "minAngle", "minimum angle to include an edge") )
{
}

void SharpLineModel::init()
{
    this->LineModel::init();

	// Triangle neighborhood construction
	if (mesh != NULL)
	{
		const int nTriangles = mesh->getNbTriangles();
		if (nTriangles != 0)
		{
			for (int i=0;i<size;i++)
			{
				elems[i].tRight = -1;
				elems[i].tLeft = -1;
				unsigned int i1 = elems[i].i1;
				unsigned int i2 = elems[i].i2;

				for (int j=0; j<nTriangles; j++)
				{
					MeshTopology::Triangle idx = mesh->getTriangle(j);
					     if ((idx[0] == i1) && (idx[1] == i2))
						elems[i].tLeft = idx[2];
					else if ((idx[1] == i1) && (idx[2] == i2))
						elems[i].tLeft = idx[0];
					else if ((idx[2] == i1) && (idx[0] == i2))
						elems[i].tLeft = idx[1];
					else if ((idx[0] == i2) && (idx[1] == i1))
						elems[i].tRight = idx[2];
					else if ((idx[1] == i2) && (idx[2] == i1))
						elems[i].tRight = idx[0];
					else if ((idx[2] == i2) && (idx[0] == i1))
						elems[i].tRight = idx[1];
				}
			}
		}
	}
	sofa::helper::vector<LineData> finalElems;
	sofa::helper::vector<SharpLineData> finalSElems;
	const VecCoord& x = *mstate->getX();
	const Real maxCosAngle = (Real)cos(minAngle.getValue()/2 * M_PI/180);
	for (int i=0;i<size;i++)
	{
		if (elems[i].tLeft >= 0 && elems[i].tRight >= 0)
		{
			Vector3 p1 = x[elems[i].i1];
			Vector3 ldir = x[elems[i].i2] - p1;
			Vector3 plane1 = cross(ldir, x[elems[i].tLeft]-p1); plane1.normalize();
			Vector3 plane2 = cross(x[elems[i].tRight]-p1, ldir); plane2.normalize();
			Vector3 n = plane1+plane2; n.normalize();
			Real cosAngle = dot(plane1,n);
			if (cosAngle < maxCosAngle)
			{
				finalElems.push_back(elems[i]);
				finalSElems.push_back(SharpLineData(n, cosAngle));
			}
			
		}
	}
	std::cout << "SharpLineModel: "<<finalElems.size() << "/" << size << " sharp edges." << std::endl;
	resize(finalElems.size());
	elems = finalElems;
	selems = finalSElems;
}

void SharpLineModel::resize(int size)
{
	this->LineModel::resize(size);
	this->selems.resize(size);
}

SharpLineModel::Coord SharpLineModel::getNormal(int i)
{
    const VecCoord& x = *mstate->getX();
    Vector3 p1 = x[elems[i].i1];
    Vector3 ldir = x[elems[i].i2] - p1;
    Vector3 plane1 = cross(ldir, x[elems[i].tLeft]-p1); plane1.normalize();
    Vector3 plane2 = cross(x[elems[i].tRight]-p1, ldir); plane2.normalize();
    Vector3 n = plane1+plane2; n.normalize();
    selems[i].normal = n;
    return n;
}

SharpLineModel::Real SharpLineModel::getCosAngle(int i)
{
    return selems[i].cosAngle;
}

void SharpLineModel::draw(int index)
{
    Line t(this,index);
    glBegin(GL_LINES);
    Vector3 p1 = t.p1();
    Vector3 p2 = t.p2();
    Vector3 dp = p2-p1;
    Vector3 c = (p1+p2)*0.5f;
    Vector3 n = getNormal(index);
    Real cosAngle = getCosAngle(index);
    Vector3 dn = defaulttype::cross(dp,n); dn.normalize();
    dn *= (Real)sqrt(1-cosAngle*cosAngle)*dp.norm();
    n *= dp.norm();
    Vector3 n1 = n*cosAngle + dn;
    Vector3 n2 = n*cosAngle - dn;
    glVertex3dv(p1.ptr());
    glVertex3dv(p2.ptr());
    glVertex3dv(c.ptr());
    glVertex3dv((c+n).ptr());
    glVertex3dv((c+n).ptr());
    glVertex3dv((c+n1).ptr());
    glVertex3dv((c+n).ptr());
    glVertex3dv((c+n2).ptr());
    glEnd();
}


void SharpLineModel::draw()
{
	if (getContext()->getShowCollisionModels())
	{
		if (getContext()->getShowWireFrame())
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		glDisable(GL_LIGHTING);
		glColor4fv(getColor4f());

		for (int i=0;i<size;i++)
		{
			//if (elems[i].i1 < elems[i].i2) // only display non-edge lines
				draw(i);
		}

		glColor3f(1.0f, 1.0f, 1.0f);
		glDisable(GL_LIGHTING);
		if (getContext()->getShowWireFrame())
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	if (getPrevious()!=NULL && getContext()->getShowBoundingCollisionModels() && dynamic_cast<core::VisualModel*>(getPrevious())!=NULL)
		dynamic_cast<core::VisualModel*>(getPrevious())->draw();
}

} // namespace collision

} // namespace component

} // namespace sofa

