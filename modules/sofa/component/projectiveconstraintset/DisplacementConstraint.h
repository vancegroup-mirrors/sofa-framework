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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_DisplacementConstraint_H
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_DisplacementConstraint_H

#include <sofa/component/projectiveconstraintset/FixedConstraint.h>

#include <sofa/helper/fixed_array.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{


using namespace sofa::helper;


/** Impose a displacement onto particles
*/
template <class DataTypes>
class DisplacementConstraint : public FixedConstraint<DataTypes>
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(DisplacementConstraint,DataTypes),SOFA_TEMPLATE(FixedConstraint, DataTypes));

   typedef typename DataTypes::VecCoord VecCoord;
   typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename DataTypes::Real Real;
	typedef topology::PointSubset SetIndex;
	typedef helper::vector<unsigned int> SetIndexArray;
	typedef defaulttype::Vec<6,Real> Vec6;

	
public:
	Data<Coord> _displacement;
	Data<Vec6> _box;
	Data<Real> _step;

	DisplacementConstraint():FixedConstraint<DataTypes>()
	, _displacement( initData(&_displacement,Coord(),"displacement","The imposed displacement") )
	, _box( initData( &_box, "box", "DOFs in the box defined by xmin,ymin,zmin, xmax,ymax,zmax are fixed") )
	, _step( initData( &_step, (Real)1, "step", "Distance at each time step") )
	{
		
	}

	virtual ~DisplacementConstraint(){};
	
	virtual void init()
	{
		FixedConstraint<DataTypes>::init();
		
		const Vec6& vb=_box.getValue();
		if( vb != Vec6() )
		{
			this->removeConstraint( 0 );
			vector <unsigned> indices;
		
			this->mstate->getIndicesInSpace( indices, vb[0],vb[3],vb[1],vb[4],vb[2],vb[5] );
			for(unsigned int i = 0; i < indices.size(); i++)
			{
				this->addConstraint(indices[i]);
				_finalPos.push_back( (*this->mstate->getX())[indices[i]]+_displacement.getValue() );
			}
		}
	}

	
	// -- Constraint interface
	virtual void projectPosition(VecCoord&x )
	{
			const SetIndexArray & indices = this->f_indices.getValue().getArray();
			int i=0;
			for (SetIndexArray::const_iterator it = indices.begin();
						it != indices.end();
						++it,++i)
			{
				if( x[*it] == _finalPos[i] ) continue; // already arrived
				
				
				Coord vecstep = _finalPos[i]-x[*it];
	
				Real distance = vecstep.norm();
	
				if(distance<_step.getValue())
				{
					x[*it] = _finalPos[i];
				}
				else
				{
					x[*it] += vecstep*(_step.getValue()/distance);
				}
			}
	}

	
	
	
	virtual void draw()
	{
		if (!this->getContext()->
				   getShowBehaviorModels()) return;
		const VecCoord& x = *this->mstate->getX();
		const VecCoord& x0 = *this->mstate->getX0();
		glDisable (GL_LIGHTING);
		glPointSize(10);
		glColor4f (0.5,0.5,1.0,1);
		
		const SetIndexArray & indices = this->f_indices.getValue().getArray();
		for (SetIndexArray::const_iterator it = indices.begin();
		it != indices.end();
		++it)
		{
			glBegin (GL_POINTS);
			gl::glVertexT(x[*it]);
			gl::glVertexT(x0[*it]);
			glEnd();
			
				glBegin (GL_LINES);
				gl::glVertexT(x[*it]);
				gl::glVertexT(x0[*it]);
				glEnd();
			
		}
		

	}
	
	
	protected:
		helper::vector<Coord> _finalPos;
	
};

} // namespace projectiveconstraintset

} // namespace component

} // namespace sofa


#endif
