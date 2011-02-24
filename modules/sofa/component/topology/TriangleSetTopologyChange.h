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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYCHANGE_H
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYCHANGE_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/topology/Topology.h>		// TopologyChange
#include <sofa/helper/vector.h>

namespace sofa
{
namespace component
{
namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Triangle Triangle;

	/** indicates that some triangles were added */
	class TrianglesAdded : public core::topology::TopologyChange  
	{
	public:
		TrianglesAdded(const unsigned int nT) 
		: core::topology::TopologyChange(core::topology::TRIANGLESADDED), 
		nTriangles(nT)
		{ }

		TrianglesAdded(const unsigned int nT, 
						const sofa::helper::vector< Triangle >& _triangleArray,
						const sofa::helper::vector< unsigned int >& trianglesIndex) 
		: core::topology::TopologyChange(core::topology::TRIANGLESADDED), 
		nTriangles(nT), 
		triangleArray(_triangleArray), 
		triangleIndexArray(trianglesIndex)
		{ }

		TrianglesAdded(const unsigned int nT, 
						const sofa::helper::vector< Triangle >& _triangleArray,
						const sofa::helper::vector< unsigned int >& trianglesIndex,
						const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
						const sofa::helper::vector< sofa::helper::vector< double > >& baryCoefs) 
		: core::topology::TopologyChange(core::topology::TRIANGLESADDED), 
		nTriangles(nT), 
		triangleArray(_triangleArray), 
		triangleIndexArray(trianglesIndex),
		ancestorsList(ancestors), 
		coefs(baryCoefs) 
		{ }

		unsigned int getNbAddedTriangles() const 
		{
			return nTriangles;
		}

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return triangleIndexArray;
		}	

		const Triangle &getTriangle(const unsigned int i)
		{
			return triangleArray[i];
		}

	public:
		unsigned int nTriangles;
		sofa::helper::vector< Triangle > triangleArray;
		sofa::helper::vector< unsigned int > triangleIndexArray;
		sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestorsList;
		sofa::helper::vector< sofa::helper::vector< double > > coefs;				
	};

	/** indicates that some triangles are about to be removed */
	class TrianglesRemoved : public core::topology::TopologyChange  
	{
	public:
		TrianglesRemoved(const sofa::helper::vector<unsigned int> _tArray) 
		: core::topology::TopologyChange(core::topology::TRIANGLESREMOVED), 
		removedTrianglesArray(_tArray) 
		{}

		unsigned int getNbRemovedTriangles() const 
		{
			return removedTrianglesArray.size();
		}

		const sofa::helper::vector<unsigned int> &getArray() const 
		{
			return removedTrianglesArray;
		}

		unsigned int &getTriangleIndices(const unsigned int i)
		{
			return removedTrianglesArray[i];
		}
	
	protected:
		sofa::helper::vector<unsigned int> removedTrianglesArray;
	};


	/** indicates that some triangles are about to be moved (i.e some/all of their vertices have just been moved)
	 * TrianglesMoved_Removing First part, remove element concerned to force object to recompute global state with current positions
	 */
	class TrianglesMoved_Removing : public core::topology::TopologyChange  
	{
	public:
	  TrianglesMoved_Removing (const sofa::helper::vector< unsigned int >& triangleShell)
	    : core::topology::TopologyChange (core::topology::TRIANGLESMOVED_REMOVING),
	    trianglesAroundVertexMoved (triangleShell)
	  {}

	public:
	  sofa::helper::vector< unsigned int > trianglesAroundVertexMoved;
	};

	
	/** indicates that some triangles are about to be moved (i.e some/all of their vertices have just been moved)
	 * TrianglesMoved_Adding Second part, recompute state of all elements previously removed, with new positions points
	 */
	class TrianglesMoved_Adding : public core::topology::TopologyChange  
	{
	public:
	  TrianglesMoved_Adding (const sofa::helper::vector< unsigned int >& triangleShell,
				 const sofa::helper::vector< Triangle >& triangleArray)
	    : core::topology::TopologyChange (core::topology::TRIANGLESMOVED_ADDING),
	    trianglesAroundVertexMoved (triangleShell), triangleArray2Moved (triangleArray)
	  {}

	public:
	  sofa::helper::vector< unsigned int > trianglesAroundVertexMoved;
	  const sofa::helper::vector< Triangle > triangleArray2Moved;
	};

	

} // namespace topology

} // namespace component

} // namespace sofa

#endif
