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
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYCHANGE_H     
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYCHANGE_H

#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>

namespace sofa
{

namespace component
{

namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Tetra Tetra;
	typedef Tetra Tetrahedron;

	/** indicates that some tetrahedra were added */
	class TetrahedraAdded : public core::topology::TopologyChange  
	{
	public:
		TetrahedraAdded(const unsigned int nT) 
		: core::topology::TopologyChange(core::topology::TETRAHEDRAADDED), 
		nTetrahedra(nT)
		{ }

		TetrahedraAdded(const unsigned int nT, 
						const sofa::helper::vector< Tetrahedron >& _tetrahedronArray,
						const sofa::helper::vector< unsigned int >& tetrahedraIndex) 
		: core::topology::TopologyChange(core::topology::TETRAHEDRAADDED), 
		nTetrahedra(nT), 
		tetrahedronArray(_tetrahedronArray), 
		tetrahedronIndexArray(tetrahedraIndex)
		{ }

		TetrahedraAdded(const unsigned int nT, 
						const sofa::helper::vector< Tetrahedron >& _tetrahedronArray,
						const sofa::helper::vector< unsigned int >& tetrahedraIndex,
						const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
						const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs) 
		: core::topology::TopologyChange(core::topology::TETRAHEDRAADDED), 
		nTetrahedra(nT), 
		tetrahedronArray(_tetrahedronArray), 
		tetrahedronIndexArray(tetrahedraIndex),
		ancestorsList(ancestors), 
		coefs(baryCoefs) 
		{ }

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return tetrahedronIndexArray;
		}

		unsigned int getNbAddedTetrahedra() const 
		{
			return nTetrahedra;
		}

	public:
		unsigned int nTetrahedra;
		sofa::helper::vector< Tetrahedron > tetrahedronArray;
		sofa::helper::vector< unsigned int > tetrahedronIndexArray;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double > > coefs;
	};

	/** indicates that some tetrahedra are about to be removed */
	class TetrahedraRemoved : public core::topology::TopologyChange  
	{
	public:
		TetrahedraRemoved(const sofa::helper::vector<unsigned int> _tArray) 
		: core::topology::TopologyChange(core::topology::TETRAHEDRAREMOVED), 
		removedTetrahedraArray(_tArray) 
		{ }

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return removedTetrahedraArray;
		}

		unsigned int getNbRemovedTetrahedra() const 
		{
			return removedTetrahedraArray.size();
		}

	public:
		sofa::helper::vector<unsigned int> removedTetrahedraArray;
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
