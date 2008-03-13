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
#ifndef SOFA_COMPONENT_RAYTRIANGLEINTERSECTION_H
#define SOFA_COMPONENT_RAYTRIANGLEINTERSECTION_H

#include <sofa/component/collision/Triangle.h>

namespace sofa
{

namespace component
{

namespace collision
{


//-----------------------------------------------------------------------------
//--IntersectionRayTriangle--
//------------------
// this class computes if a Triangle P intersects a line segment
//-----------------------------------------------------------------------------

class RayTriangleIntersection
{
public:
	RayTriangleIntersection(); // start a Proximity solver
	~RayTriangleIntersection();

	// init the solver with the new coordinates of the triangle & the segment
	// solve the lcp
	bool NewComputation( Triangle *triP, const sofa::defaulttype::Vector3 &origin, const sofa::defaulttype::Vector3 &direction,  double &t,  double &u, double &v);

	//double getAlphaP(){return _result[6];}
	//double getBetaP(){return _result[7];}
	//double getAlphaQ(){return _result[8];}
	//double getBetaQ(){return _result[9];}


private:
	//double **_A;
	//double *_b; 
	//double *_result;
};

}
}
}
#endif
