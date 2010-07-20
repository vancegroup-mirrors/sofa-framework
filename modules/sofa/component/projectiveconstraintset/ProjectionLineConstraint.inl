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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionLineConstraint_INL
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionLineConstraint_INL

#include <sofa/component/projectiveconstraintset/ProjectionLineConstraint.h>

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
ProjectionLineConstraint<DataTypes>::ProjectionLineConstraint()
: core::behavior::ProjectiveConstraintSet<DataTypes>(NULL)
, _indices( initData(&_indices,"indices","Indices of the projected points") )
		,_segment( initData(&_segment,false,"isSegment","Is the constrained line a segment? (given points are projected onto the segment)") )
		, _line( initData(&_line,fixed_array<int,2>(),"line","the line where project the given point") )
				, _bilateral( initData(&_bilateral,false,"bilateral","Does a force applied on a constrained point influence the line/segment points?") )		
{
}


template <class DataTypes>
ProjectionLineConstraint<DataTypes>::~ProjectionLineConstraint()
{
}






// -- Constraint interface


template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::init()
{
    this->core::behavior::ProjectiveConstraintSet<DataTypes>::init();
	
	_coords.resize( _indices.getValue().getArray().size() );
}

template <class DataTypes> template <class DataDeriv>
void ProjectionLineConstraint<DataTypes>::projectResponseT(DataDeriv& res)
{
    const SetIndexArray & indices = _indices.getValue().getArray();
	int i=0;
	 for (SetIndexArray::const_iterator it = indices.begin();
			  it != indices.end();
			  ++it,++i)
	 {
		 Deriv old = res[*it];
		 
		 
		 if( _segment.getValue() && _out )
			 res[*it] = Deriv();
		 else
			 res[*it] = _AB * (_AB * res[*it] / _sqnormAB);
		 
		 
		 if( _bilateral.getValue() )
		 {
			 Deriv projectiononnormal = old-res[*it];
			 res[ _line.getValue()[0] ] += projectiononnormal*(1-_coords[i]);
			 res[ _line.getValue()[1] ] += projectiononnormal*_coords[i];
		 }
	 }
}
template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::projectResponse(VecDeriv& res)
{
  projectResponseT(res);
}
template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::projectResponse(SparseVecDeriv& res)
{
  projectResponseT(res);
}

template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::projectVelocity(VecDeriv& dx)
{
	const SetIndexArray & indices = _indices.getValue().getArray();
	for (SetIndexArray::const_iterator it = indices.begin();
			 it != indices.end();
			 ++it)
	{
		Real t=_AB * dx[*it] / _sqnormAB;
		if( _segment.getValue() )
		{
			switch(_out)
			{
				case 1:
					 if(t<0)
					t=0.0;
					break;
				case 2 :
					 if(t>0)
					t=0.0;
					break;
			}
		}
		dx[*it] = _AB*t;
	}
}
template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::projectPosition(VecCoord& x)
{
	const SetIndexArray & indices = _indices.getValue().getArray();
	
	_A = x[ _line.getValue()[0]];
	_B = x[ _line.getValue()[1]];
	_AB = _B-_A;
	_sqnormAB = _AB.norm2();
	
	int i=0;
	for (SetIndexArray::const_iterator it = indices.begin();
			 it != indices.end();
			 ++it,++i)
	{
		_coords[i] = (float)(_AB * (x[*it]-_A) / _sqnormAB );
		if( _segment.getValue() )
		{
			if( _coords[i]<0 )
			{
				_coords[i]=0.0;
				_out=1; // trop a gauche
			}
			else if( _coords[i]>1)
			{
				_coords[i]=1.0;
				_out=2; //trop a droite
			}
			else _out=0;
		}
		x[*it] = _A + _AB*_coords[i];
	}
}


template <class DataTypes>
void ProjectionLineConstraint<DataTypes>::draw()
{
    if (!this->getContext()->
            getShowBehaviorModels()) return;
    const VecCoord& x = *this->mstate->getX();
    //serr<<"ProjectionLineConstraint<DataTypes>::draw(), x.size() = "<<x.size()<<sendl;
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
	
	glLineWidth(1);
	if(_segment.getValue())
	{
		glBegin (GL_LINES);
		gl::glVertexT(_A);
		gl::glVertexT(_B);
		glEnd();
	}
	else
	{
		glBegin (GL_LINES);
		gl::glVertexT(_A-_AB*.3);
		gl::glVertexT(_B+_AB*.3);
		glEnd();
	}	
}



} // namespace constraint

} // namespace component

} // namespace sofa

#endif


