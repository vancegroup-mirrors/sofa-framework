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
#ifndef SOFA_COMPONENT_TOPOLOGY_QUADSETTOPOLOGYCHANGE_H
#define SOFA_COMPONENT_TOPOLOGY_QUADSETTOPOLOGYCHANGE_H

#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>

namespace sofa
{
namespace component
{
namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Quad Quad;
    
	/** indicates that some quads were added */
	class QuadsAdded : public core::topology::TopologyChange  
	{
	public:
		QuadsAdded(const unsigned int nT) 
		: core::topology::TopologyChange(core::topology::QUADSADDED), 
		nQuads(nT)
		{ }

		QuadsAdded(const unsigned int nT, 
					const sofa::helper::vector< Quad >& _quadArray,
					const sofa::helper::vector< unsigned int >& quadsIndex) 
		: core::topology::TopologyChange(core::topology::QUADSADDED), 
		nQuads(nT), 
		quadArray(_quadArray), 
		quadIndexArray(quadsIndex)
		{ }

		QuadsAdded(const unsigned int nT, 
					const sofa::helper::vector< Quad >& _quadArray,
					const sofa::helper::vector< unsigned int >& quadsIndex,
					const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
					const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs) 
		: core::topology::TopologyChange(core::topology::QUADSADDED), 
		nQuads(nT), 
		quadArray(_quadArray), 
		quadIndexArray(quadsIndex),
		ancestorsList(ancestors), 
		coefs(baryCoefs) 
		{ }

		unsigned int getNbAddedQuads() const 
		{
			return nQuads;
		}

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return quadIndexArray;
		}	

		const Quad &getQuad(const unsigned int i) const
		{
			return quadArray[i];
		}

	public:
		unsigned int nQuads;
		sofa::helper::vector< Quad > quadArray;
		sofa::helper::vector< unsigned int > quadIndexArray;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double > > coefs;
	};

	/** indicates that some quads are about to be removed */
	class QuadsRemoved : public core::topology::TopologyChange  
	{
	public:
		QuadsRemoved(const sofa::helper::vector<unsigned int> _qArray)
		: core::topology::TopologyChange(core::topology::QUADSREMOVED), 
		removedQuadsArray(_qArray) 
		{ }

		unsigned int getNbRemovedQuads() const 
		{
			return removedQuadsArray.size();
		}

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return removedQuadsArray;
		}	

		unsigned int &getQuadIndices(const unsigned int i)
		{
			return removedQuadsArray[i];
		}

	protected:
		sofa::helper::vector<unsigned int> removedQuadsArray;	
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
