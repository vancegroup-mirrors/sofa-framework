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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYCHANGE_H
#define SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYCHANGE_H

#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>

namespace sofa
{

namespace component
{

namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Edge Edge;

	/** indicates that some edges were added */
	class EdgesAdded : public core::topology::TopologyChange  
	{
    public:
		EdgesAdded(const unsigned int nE) 
			: core::topology::TopologyChange(core::topology::EDGESADDED), 
			nEdges(nE)
		{ }

		EdgesAdded(const unsigned int nE, 
			const sofa::helper::vector< Edge >& edgesList,
			const sofa::helper::vector< unsigned int >& edgesIndex) 
			: core::topology::TopologyChange(core::topology::EDGESADDED), 
			nEdges(nE), 
			edgeArray(edgesList), 
			edgeIndexArray(edgesIndex)
		{ }

		EdgesAdded(const unsigned int nE, 
			const sofa::helper::vector< Edge >& edgesList,
			const sofa::helper::vector< unsigned int >& edgesIndex,
			const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors) 
			: core::topology::TopologyChange(core::topology::EDGESADDED), 
			nEdges(nE), 
			edgeArray(edgesList), 
			edgeIndexArray(edgesIndex), 
			ancestorsList(ancestors)
		{ }

		EdgesAdded(const unsigned int nE, 
			const sofa::helper::vector< Edge >& edgesList,
			const sofa::helper::vector< unsigned int >& edgesIndex,
			const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
			const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs) 
			: core::topology::TopologyChange(core::topology::EDGESADDED), 
			nEdges(nE), 
			edgeArray(edgesList), 
			edgeIndexArray(edgesIndex), 
			ancestorsList(ancestors), 
			coefs(baryCoefs) 
		{ }

		virtual ~EdgesAdded() {}

		unsigned int getNbAddedEdges() const { return nEdges;}
	/*	const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return edgeIndexArray;
		}*/
		const sofa::helper::vector< Edge > &getArray() const 
		{
			return edgeArray;
		}

	public:
		unsigned int nEdges;
		sofa::helper::vector< Edge > edgeArray;
		sofa::helper::vector< unsigned int > edgeIndexArray;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double > > coefs;
	};


	/** indicates that some edges are about to be removed */
	class EdgesRemoved : public core::topology::TopologyChange  
	{
	public:
		EdgesRemoved(const sofa::helper::vector<unsigned int> _eArray)
		: core::topology::TopologyChange(core::topology::EDGESREMOVED), 
		removedEdgesArray(_eArray) 
		{}

		~EdgesRemoved() {}

		virtual const sofa::helper::vector<unsigned int> &getArray() const 
		{ 
			return removedEdgesArray; 
		}

		virtual unsigned int getNbRemovedEdges() const 
		{ 
			return removedEdgesArray.size(); 
		}

	public:
		sofa::helper::vector<unsigned int> removedEdgesArray;
	};


	/** indicates that some edges are about to be moved (i.e one or both of their vertices have just been moved)
	 * EdgesMoved_Removing First part, remove element concerned to force object to recompute global state with current positions
	 */
	class EdgesMoved_Removing : public core::topology::TopologyChange  
	{
	public:
	  EdgesMoved_Removing (const sofa::helper::vector< unsigned int >& edgeShell)
	    : core::topology::TopologyChange (core::topology::EDGESMOVED_REMOVING),
	    edgesAroundVertexMoved (edgeShell)
	  {}

	public:
	  sofa::helper::vector< unsigned int > edgesAroundVertexMoved;
	};

	
	/** indicates that some edges are about to be moved (i.e one or both of their vertices have just been moved)
	 * EdgesMoved_Adding Second part, recompute state of all elements previously removed, with new positions points
	 */
	
	class EdgesMoved_Adding : public core::topology::TopologyChange  
	{
	public:
	  EdgesMoved_Adding (const sofa::helper::vector< unsigned int >& edgeShell,
			     const sofa::helper::vector< Edge >& edgeArray)
	    : core::topology::TopologyChange (core::topology::EDGESMOVED_ADDING),
	    edgesAroundVertexMoved (edgeShell), edgeArray2Moved (edgeArray)
	  {}

	public:
	  sofa::helper::vector< unsigned int > edgesAroundVertexMoved;
	  sofa::helper::vector< Edge > edgeArray2Moved;
	};

	

} // namespace topology

} // namespace component

} // namespace sofa

#endif
