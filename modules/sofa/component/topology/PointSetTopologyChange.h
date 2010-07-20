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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYCHANGE_H
#define SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYCHANGE_H

#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>
#include <sofa/component/component.h>

namespace sofa
{

namespace component
{

namespace topology
{

	/** indicates that the indices of two points are being swapped */
	class PointsIndicesSwap : public core::topology::TopologyChange
	{
	public:
		PointsIndicesSwap(const unsigned int i1,const unsigned int i2)
		: core::topology::TopologyChange(core::topology::POINTSINDICESSWAP)
		{
			index[0]=i1;
			index[1]=i2;
		}

	public:
		unsigned int index[2];
	};

	/** indicates that some points were added */
	class PointsAdded : public core::topology::TopologyChange
	{
	public:

		PointsAdded(const unsigned int nV)
		: core::topology::TopologyChange(core::topology::POINTSADDED)
		, nVertices(nV)
		{ }

		PointsAdded(const unsigned int nV,
					const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
					const sofa::helper::vector< sofa::helper::vector< double       > >& baryCoefs)
		: core::topology::TopologyChange(core::topology::POINTSADDED)
		, nVertices(nV), ancestorsList(ancestors), coefs(baryCoefs)
		{ }

		unsigned int getNbAddedVertices() const {return nVertices;}

	public:
		unsigned int nVertices;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double       > > coefs;
	};

	/** indicates that some points are about to be removed */
	class PointsRemoved : public core::topology::TopologyChange
	{
	public:
		PointsRemoved(const sofa::helper::vector<unsigned int>& _vArray)
		: core::topology::TopologyChange(core::topology::POINTSREMOVED),
		removedVertexArray(_vArray)
		{ }

		const sofa::helper::vector<unsigned int> &getArray() const { return removedVertexArray;	}

	public:
		sofa::helper::vector<unsigned int> removedVertexArray;
	};


	/** indicates that the indices of all points have been renumbered */
	class PointsRenumbering : public core::topology::TopologyChange
	{
	public:

		PointsRenumbering()
			: core::topology::TopologyChange(core::topology::POINTSRENUMBERING)
		{ }

		PointsRenumbering(const sofa::helper::vector< unsigned int >& indices,
						  const sofa::helper::vector< unsigned int >& inv_indices)
			: core::topology::TopologyChange(core::topology::POINTSRENUMBERING),
			indexArray(indices), inv_indexArray(inv_indices)
		{ }

		const sofa::helper::vector<unsigned int> &getIndexArray() const { return indexArray; }

		const sofa::helper::vector<unsigned int> &getinv_IndexArray() const { return inv_indexArray; }

	public:
		sofa::helper::vector<unsigned int> indexArray;
		sofa::helper::vector<unsigned int> inv_indexArray;
	};


	/** indicates that some points were moved */
	class PointsMoved : public core::topology::TopologyChange
	{
	public:

	  PointsMoved(const sofa::helper::vector<unsigned int>& indices,
                      const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
                      const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs)
	  : core::topology::TopologyChange(core::topology::POINTSMOVED)
	  , indicesList(indices), ancestorsList(ancestors), baryCoefsList(baryCoefs)
	  {}

	public:
          sofa::helper::vector<unsigned int> indicesList;
          sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
          sofa::helper::vector< sofa::helper::vector< double > > baryCoefsList;
	};


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENTS_POINTSETTOPOLOGYCHANGE_H
