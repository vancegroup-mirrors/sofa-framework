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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_POINTDATA_INL

#include <sofa/component/topology/PointData.h>

#include <sofa/component/topology/PointSetTopologyChange.h>
#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

namespace sofa
{

namespace component
{

namespace topology
{

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////implementation//////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template <typename T, typename VecT>
	void PointData<T,VecT>::handleTopologyEvents( std::list<  const core::topology::TopologyChange *>::const_iterator changeIt, 
												std::list< const core::topology::TopologyChange *>::const_iterator &end ) 
	{
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
					(*m_createTetrahedronFunc)(ea->tetrahedronIndexArray,m_createParam, *(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::topology::TETRAHEDRAREMOVED:
			 {
				 if (m_destroyTetrahedronFunc) 
				 {
					const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
					(*m_destroyTetrahedronFunc)(er->getArray(),m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }		
			 case core::topology::TRIANGLESADDED:
			 {
				 if (m_createTriangleFunc) 
				 {
					const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
					(*m_createTriangleFunc)(ea->triangleIndexArray,m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::topology::TRIANGLESREMOVED:
			 {
				 if (m_destroyTriangleFunc) 
				 {
					const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
					(*m_destroyTriangleFunc)(er->getArray(),m_createParam, *(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }						 
			 case core::topology::EDGESADDED:
			 {
				 if (m_createEdgeFunc) 
				 {
					const EdgesAdded *ea=static_cast< const EdgesAdded* >( *changeIt );
					(*m_createEdgeFunc)(ea->edgeIndexArray,m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::topology::EDGESREMOVED:
			 {
				 if (m_destroyEdgeFunc) 
				 {
					const EdgesRemoved *er=static_cast< const EdgesRemoved * >( *changeIt );
					(*m_destroyEdgeFunc)(er->getArray(),m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::topology::POINTSINDICESSWAP:
			 {
				 unsigned int i1 = ( static_cast< const PointsIndicesSwap * >( *changeIt ) )->index[0];
				 unsigned int i2 = ( static_cast< const PointsIndicesSwap* >( *changeIt ) )->index[1];
				 swap( i1, i2 );
				 break;
			 }
			 case core::topology::POINTSADDED:
			 {
				 unsigned int nbPoints = ( static_cast< const PointsAdded * >( *changeIt ) )->getNbAddedVertices();
				 sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( static_cast< const PointsAdded * >( *changeIt ) )->ancestorsList;
				 sofa::helper::vector< sofa::helper::vector< double       > > coefs     = ( static_cast< const PointsAdded * >( *changeIt ) )->coefs;
				 add( nbPoints, ancestors, coefs );
				 break;
			 }
			 case core::topology::POINTSREMOVED:
			 {
				 const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRemoved * >( *changeIt ) )->getArray();
				 remove( tab );
				 break;
			 }
			 case core::topology::POINTSRENUMBERING:
			 {

				 const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRenumbering * >( *changeIt ) )->getIndexArray(); 
				 renumber( tab );
				 break;
			 }
			 case core::topology::POINTSMOVED:
			 {
			   const sofa::helper::vector< unsigned int >& indexList = ( static_cast< const PointsMoved * >( *changeIt ) )->indicesList;
			   const sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( static_cast< const PointsMoved * >( *changeIt ) )->ancestorsList;
			   sofa::helper::vector< sofa::helper::vector< double > > coefs = ( static_cast< const PointsMoved * >( *changeIt ) )->baryCoefsList;
			   container_type& data = *(this->beginEdit());

			   for (unsigned int i = 0; i <indexList.size(); i++)
			   {
			     m_destroyFunc( indexList[i], m_destroyParam, data[indexList[i]] );  
			     m_createFunc( indexList[i], m_createParam, data[indexList[i]], ancestors[i], coefs[i] );
			   }

			   this->endEdit();
			   break;
			 }
			 case core::topology::EDGESMOVED_REMOVING:
			 {
			   if (m_destroyEdgeFunc) 
			   {
			     const EdgesMoved_Removing *em=static_cast< const EdgesMoved_Removing * >( *changeIt );
			     (*m_destroyEdgeFunc)(em->edgesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
			     this->endEdit();
			   }

			   break;
			 }
			 case core::topology::EDGESMOVED_ADDING:
			 {
			   if (m_createEdgeFunc) 
			   {
			     const EdgesMoved_Adding *em=static_cast< const EdgesMoved_Adding* >( *changeIt );
			     (*m_createEdgeFunc)(em->edgesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
			     this->endEdit();
			   }

			   break;
			 }
			 case core::topology::TRIANGLESMOVED_REMOVING:
			 {
			   if (m_destroyTriangleFunc) 
			   {
			     const TrianglesMoved_Removing *tm=static_cast< const TrianglesMoved_Removing* >( *changeIt );
			     (*m_destroyTriangleFunc)(tm->trianglesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
			     this->endEdit();
			   }
			   
			   break;
			 }
			 case core::topology::TRIANGLESMOVED_ADDING:
			 {
			   if (m_createTriangleFunc) 
			   {
			     const TrianglesMoved_Adding *tm=static_cast< const TrianglesMoved_Adding * >( *changeIt );
			     (*m_createTriangleFunc)(tm->trianglesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
			     this->endEdit();
			   }

			   break;
			 }
			 default:
				 // Ignore events that are not point related.
				 break;
			}; // switch( changeType )

			++changeIt;
		}
	}

	template <typename T, typename VecT>
	void PointData<T,VecT>::swap( unsigned int i1, unsigned int i2 ) 
	{
		container_type& data = *(this->beginEdit());

		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;

		this->endEdit();
	}

	template <typename T, typename VecT>
	void PointData<T,VecT>::add( unsigned int nbPoints, 
								const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
								const sofa::helper::vector< sofa::helper::vector< double > >& coefs) 
	{
		container_type& data = *(this->beginEdit());

		// Using default values
		unsigned int i0 = data.size();
		data.resize(i0+nbPoints);

		for (unsigned int i = 0; i < nbPoints; ++i)
		{
			T& t = data[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc( i0+i, m_createParam, t, empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc( i0+i, m_createParam, t, ancestors[i], coefs[i] );
		}

		this->endEdit();
	}



	template <typename T, typename VecT>
	void PointData<T,VecT>::remove( const sofa::helper::vector<unsigned int> &index ) 
	{
		unsigned int last = this->getValue().size() -1;

		container_type& data = *(this->beginEdit());

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			m_destroyFunc( index[i], m_destroyParam, data[index[i]] );
			swap( index[i], last );
			--last;
		}

		data.resize( data.size() - index.size() );
		this->endEdit();
	}


	template <typename T, typename VecT>
	void PointData<T,VecT>::renumber( const sofa::helper::vector<unsigned int> &index ) 
	{
		container_type& data = *(this->beginEdit());

		container_type copy = this->getValue(); // not very efficient memory-wise, but I can see no better solution...
		for (unsigned int i = 0; i < index.size(); ++i)
		{
			data[i] = copy[ index[i] ];

		}

		this->endEdit();
	}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _POINTDATA_INL_
