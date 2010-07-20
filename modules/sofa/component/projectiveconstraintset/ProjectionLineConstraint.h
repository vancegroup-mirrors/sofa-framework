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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionLineConstraint_H
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectionLineConstraint_H

#include <sofa/core/behavior/ProjectiveConstraintSet.h>
#include <sofa/component/topology/PointSubset.h>

#include <sofa/helper/fixed_array.h>

namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{


using namespace sofa::helper;


/** Project given particles on a line/segment defined by others particles.
*/
template <class DataTypes>
class ProjectionLineConstraint : public core::behavior::ProjectiveConstraintSet<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ProjectionLineConstraint,DataTypes),SOFA_TEMPLATE(core::behavior::ProjectiveConstraintSet, DataTypes));

   typedef typename DataTypes::VecCoord VecCoord;
   typedef typename DataTypes::VecDeriv VecDeriv;
   typedef typename DataTypes::SparseVecDeriv SparseVecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename DataTypes::Real Real;
	typedef topology::PointSubset SetIndex;
	typedef helper::vector<unsigned int> SetIndexArray;


	
public:
	Data<SetIndex> _indices;
	Data<bool> _segment;
	Data<fixed_array<int,2> > _line;
	Data<bool> _bilateral;

	ProjectionLineConstraint();

	virtual ~ProjectionLineConstraint();

	
	// -- Constraint interface
	void init();
    template <class DataDeriv>
        void projectResponseT(DataDeriv& dx);

    void projectResponse(VecDeriv& dx);
    void projectResponse(SparseVecDeriv& dx);
	virtual void projectVelocity(VecDeriv& ); ///< project dx to constrained space (dx models a velocity)
	virtual void projectPosition(VecCoord& ); ///< project x to constrained space (x models a position)

	virtual void draw();
		

protected :

	sofa::core::topology::BaseMeshTopology* topology;
	
	Coord _A,_B,_AB;
	Real _sqnormAB;
	unsigned _out;
	helper::vector<float> _coords;

};

} // namespace projectiveconstraintset

} // namespace component

} // namespace sofa


#endif
