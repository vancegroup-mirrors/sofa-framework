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
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYCHANGE_H
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYCHANGE_H

#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>

namespace sofa
{
namespace component
{
namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Hexa Hexa;
	typedef Hexa Hexahedron;

	/** indicates that some hexahedra were added */
	class HexahedraAdded : public core::topology::TopologyChange  
	{
	public:
		HexahedraAdded(const unsigned int nT)
		: core::topology::TopologyChange(core::topology::HEXAHEDRAADDED), 
		nHexahedra(nT)
		{ }

		HexahedraAdded(const unsigned int nT, 
					const sofa::helper::vector< Hexahedron >& _hexahedronArray,
					const sofa::helper::vector< unsigned int >& hexahedraIndex) 
		: core::topology::TopologyChange(core::topology::HEXAHEDRAADDED), 
		nHexahedra(nT), 
		hexahedronArray(_hexahedronArray), 
		hexahedronIndexArray(hexahedraIndex)
		{ }

		HexahedraAdded(const unsigned int nT, 
					const sofa::helper::vector< Hexahedron >& _hexahedronArray,
					const sofa::helper::vector< unsigned int >& hexahedraIndex,
					const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
					const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs) 
		: core::topology::TopologyChange(core::topology::HEXAHEDRAADDED), 
		nHexahedra(nT), 
		hexahedronArray(_hexahedronArray), 
		hexahedronIndexArray(hexahedraIndex),
		ancestorsList(ancestors), 
		coefs(baryCoefs) 
		{ }

		unsigned int getNbAddedHexahedra() const 
		{
			return nHexahedra;
		}

	public:
		unsigned int nHexahedra;
		sofa::helper::vector< Hexahedron > hexahedronArray;
		sofa::helper::vector< unsigned int > hexahedronIndexArray;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double > > coefs;
	};

	/** indicates that some hexahedra are about to be removed */
	class HexahedraRemoved : public core::topology::TopologyChange  
	{
	public:
		HexahedraRemoved(const sofa::helper::vector<unsigned int> _tArray)
		: core::topology::TopologyChange(core::topology::HEXAHEDRAREMOVED), 
		removedHexahedraArray(_tArray) 
		{ }

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return removedHexahedraArray;
		}	

		unsigned int getNbRemovedHexahedra() const 
		{
			return removedHexahedraArray.size();
		}

	public:
		sofa::helper::vector<unsigned int> removedHexahedraArray;
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
