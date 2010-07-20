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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionPlaneConstraint_INL
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionPlaneConstraint_INL

#include <sofa/component/projectiveconstraintset/ProjectionPlaneConstraint.h>

#include <sofa/helper/gl/template.h>






namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{

using namespace core::topology;

using namespace sofa::defaulttype;
using namespace sofa::core::behavior;



template <class DataTypes>
ProjectionPlaneConstraint<DataTypes>::ProjectionPlaneConstraint()
: core::behavior::ProjectiveConstraintSet<DataTypes>(NULL)
, _indices( initData(&_indices,"indices","Indices of the projected points") )
		, _plane( initData(&_plane,fixed_array<int,3>(),"plane","indices of points giving the plane where are projected the constrained points") )
// 		, _bilateral( initData(&_bilateral,false,"bilateral","Does a force applied on a constrained point influence the plane's points?") )	
{
}


template <class DataTypes>
ProjectionPlaneConstraint<DataTypes>::~ProjectionPlaneConstraint()
{
}






// -- Constraint interface


template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::init()
{
    this->core::behavior::ProjectiveConstraintSet<DataTypes>::init();
	 
// 	_coords.resize( _indices.getValue().getArray().size() );
}

template <class DataTypes> template <class DataDeriv>
void ProjectionPlaneConstraint<DataTypes>::projectResponseT(DataDeriv& res)
{
    const SetIndexArray & indices = _indices.getValue().getArray();
// 	int i=0;
	 for (SetIndexArray::const_iterator it = indices.begin();
			  it != indices.end();
			  ++it/*,++i*/)
	 {
		 const Deriv& P = res[*it];
		 Real t = (-a*P[0]-b*P[1]-c*P[2]-d)/constant;
	
		 Deriv projectiononnormal;
		 projectiononnormal[0] = t*a;
		 projectiononnormal[1] = t*b;
		 projectiononnormal[2] = t*c;
		 
		 res[*it] += projectiononnormal;
		 
// 		 if( _bilateral.getValue() )
// 		 {
// 			 res[ _plane.getValue()[0] ] += projectiononnormal * (1-_coords[i][0]-_coords[i][1]);
// 			 res[ _plane.getValue()[1] ] += projectiononnormal * _coords[i][0];
// 			 res[ _plane.getValue()[2] ] += projectiononnormal * _coords[i][1];
// 		 }
	 }
}

template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::projectResponse(VecDeriv& res)
{
  projectResponseT(res);
}

template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::projectResponse(SparseVecDeriv& res)
{
  projectResponseT(res);
}

template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::projectVelocity(VecDeriv& dx)
{
	const SetIndexArray & indices = _indices.getValue().getArray();
	for (SetIndexArray::const_iterator it = indices.begin();
			 it != indices.end();
			 ++it)
	{
		Deriv P = dx[*it];
		Real t = (-a*P[0]-b*P[1]-c*P[2]-d)/constant;
	
		dx[*it][0] = t*a;
		dx[*it][1] = t*b;
		dx[*it][2] = t*c;
		dx[*it] += P;
	}
}
template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::projectPosition(VecCoord& x)
{
	const SetIndexArray & indices = _indices.getValue().getArray();
	
	
		 // find the plane, cartesian equation 	ax+by+cz+d=0
	Coord A = x[ _plane.getValue()[0]];
	Coord B = x[ _plane.getValue()[1]];
	Coord C = x[ _plane.getValue()[2]];
	
	
	Coord AB = B-A;
	Coord AC = C-A;
	
	a = AB[1]*AC[2] - AB[2]*AC[1];
	b = -AB[0]*AC[2] + AB[2]*AC[0];
	c = AB[0]*AC[1] - AB[1]*AC[0];
	d = -A[0]*a-A[1]*b-A[2]*c;
	
	constant = a*a+b*b+c*c;


// 	int i=0;
	for (SetIndexArray::const_iterator it = indices.begin();
			 it != indices.end();
			 ++it/*,++i*/)
	{
		
		Real t = (-a*x[*it][0]-b*x[*it][1]-c*x[*it][2]-d)/constant;
	
		x[*it][0] += t*a;
		x[*it][1] += t*b;
		x[*it][2] += t*c;
		
// 		if( _bilateral.getValue() )
// 		{
// 			// find barycentric coordinates in triangle
// 			_coords[i][0] = sqrt(cross(AC, x[*it]).norm2() / constant);
// 			_coords[i][1] = sqrt(cross(AB, x[*it]).norm2() / constant);
// 		} 
	}
}


template <class DataTypes>
void ProjectionPlaneConstraint<DataTypes>::draw()
{
    if (!this->getContext()->
            getShowBehaviorModels()) return;
    const VecCoord& x = *this->mstate->getX();
    //serr<<"ProjectionPlaneConstraint<DataTypes>::draw(), x.size() = "<<x.size()<<sendl;
    glDisable (GL_LIGHTING);
    glPointSize(10);
    glColor4f (0.5f,0.5f,0.9f,1.0f);
    glBegin (GL_POINTS);
	const SetIndexArray & indices = _indices.getValue().getArray();
	for (SetIndexArray::const_iterator it = indices.begin();
			 it != indices.end();
			 ++it)
	{
		gl::glVertexT(x[*it]);
	}
		
    glEnd();
	 
	 
	 Coord A = x[ _plane.getValue()[0]];
	 Coord B = x[ _plane.getValue()[1]];
	 Coord C = x[ _plane.getValue()[2]];
	
	glLineWidth(1);
	glBegin (GL_LINES);
	gl::glVertexT(A);
	gl::glVertexT(B);
	
	gl::glVertexT(A);
	gl::glVertexT(C);
	
	gl::glVertexT(C);
	gl::glVertexT(B);
	
	glEnd();

}



} // namespace constraint

} // namespace component

} // namespace sofa

#endif


