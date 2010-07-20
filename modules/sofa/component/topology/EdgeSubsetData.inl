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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESUBSETDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_EDGESUBSETDATA_INL

#include <sofa/component/topology/EdgeSubsetData.h>
#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>	
#include <sofa/component/topology/TetrahedronSetTopologyChange.h> 

namespace sofa
{

namespace component
{

namespace topology
{

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////implementation//////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////


	template <typename T>
	void EdgeSubsetData<T>::handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
												std::list< const core::topology::TopologyChange *>::const_iterator &end,
												const unsigned int totalEdgeSetArraySize) 
	{
		setTotalEdgeSetArraySize(totalEdgeSetArraySize);

		while( changeIt != end )
		{
			core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
			 case core::topology::TETRAHEDRAADDED:
			 {
				 if (m_createTetrahedronFunc) 
				 {
					 const TetrahedraAdded *ea=static_cast< const TetrahedraAdded* >( *changeIt );
					 (*m_createTetrahedronFunc)(ea->tetrahedronIndexArray,m_createParam,*this);
				 }
				 break;
			 }
			 case core::topology::TETRAHEDRAREMOVED:
			 {
				 if (m_destroyTetrahedronFunc) 
				 {
					 const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
					 (*m_destroyTetrahedronFunc)(er->getArray(),m_createParam,*this);
				 }
				 break;
			 }		
			 case core::topology::TRIANGLESADDED:
			 {
				 if (m_createTriangleFunc) 
				 {
					 const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
					 (*m_createTriangleFunc)(ea->triangleIndexArray,m_createParam,*this);
				 }
				 break;
			 }
			 case core::topology::TRIANGLESREMOVED:
			 {
				 if (m_destroyTriangleFunc) 
				 {
					 const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
					 (*m_destroyTriangleFunc)(er->getArray(),m_createParam,*this);
				 }
				 break;
			 }	
			 case core::topology::EDGESADDED:
			 {
				 const EdgesAdded *ea=static_cast< const EdgesAdded * >( *changeIt );
				 add( ea->getNbAddedEdges(), ea->edgeArray, ea->ancestorsList, ea->coefs );
				 break;
			 }
			 case core::topology::EDGESREMOVED:
			 {
				 const std::vector<unsigned int> &tab = ( static_cast< const EdgesRemoved *>( *changeIt ) )->getArray();
				 remove( tab );
				 break;
			 }
			 default:
				 // Ignore events that are not Edge or Point related.
				 break;
			}; // switch( changeType )

			++changeIt;
		}
	}

	template <typename T>
	void EdgeSubsetData<T>::add( unsigned int nbEdges, 
								const sofa::helper::vector< Edge >& edge, 
								const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
								const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		// Using default values
		unsigned int s = this->size();

		for (unsigned int i = 0; i < nbEdges; ++i)
		{
			T t;
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc( s + i, m_createParam, t, edge[i], empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc( s + i, m_createParam, t, edge[i], ancestors[i], coefs[i] );

			// test that there are no item with the key = s+i
			assert(this->find(s+i)==this->end());
			insert(std::pair<unsigned int,T>(s+i,t));
		}
		lastEdgeIndex+=nbEdges;
	}

	template <typename T>
	void EdgeSubsetData<T>::remove( const std::vector<unsigned int> &index ) 
	{
		typename std::map< unsigned int, T >::iterator it,itend;

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			it=this->find(index[i]);
			if (it!=this->end()) 
			{
				m_destroyFunc( index[i], m_destroyParam, (*this)[index[i]] );
				/// the edge lastEdgeIndex is now set to index index[i]
				itend=this->find(lastEdgeIndex);

				if (itend!=this->end()) 
				{
					/// change the key of the item associated with key lastEdgeIndex 
					T tmp=(*itend).second;
					erase(itend);
					insert(std::pair<unsigned int,T>(index[i],tmp));
				}
				erase(it);
			}
			--lastEdgeIndex;
		}
		//sout << "EdgeSubsetData: vector has now "<<this->size()<<" entries."<<sendl;
	}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _EdgeDATA_INL_
