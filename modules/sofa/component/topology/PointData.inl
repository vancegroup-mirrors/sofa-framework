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

	template <typename T, typename Alloc>
	void PointData<T,Alloc>::handleTopologyEvents( std::list<  const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, 
												std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
	{
		while( changeIt != end )
		{
			core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
			 case core::componentmodel::topology::TETRAHEDRAADDED:
			 {
				 if (m_createTetrahedronFunc) 
				 {
					const TetrahedraAdded *ea=static_cast< const TetrahedraAdded* >( *changeIt );
					(*m_createTetrahedronFunc)(ea->tetrahedronIndexArray,m_createParam, *(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::componentmodel::topology::TETRAHEDRAREMOVED:
			 {
				 if (m_destroyTetrahedronFunc) 
				 {
					const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
					(*m_destroyTetrahedronFunc)(er->getArray(),m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }		
			 case core::componentmodel::topology::TRIANGLESADDED:
			 {
				 if (m_createTriangleFunc) 
				 {
					const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
					(*m_createTriangleFunc)(ea->triangleIndexArray,m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::componentmodel::topology::TRIANGLESREMOVED:
			 {
				 if (m_destroyTriangleFunc) 
				 {
					const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
					(*m_destroyTriangleFunc)(er->getArray(),m_createParam, *(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }						 
			 case core::componentmodel::topology::EDGESADDED:
			 {
				 if (m_createEdgeFunc) 
				 {
					const EdgesAdded *ea=static_cast< const EdgesAdded* >( *changeIt );
					(*m_createEdgeFunc)(ea->edgeIndexArray,m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::componentmodel::topology::EDGESREMOVED:
			 {
				 if (m_destroyEdgeFunc) 
				 {
					const EdgesRemoved *er=static_cast< const EdgesRemoved * >( *changeIt );
					(*m_destroyEdgeFunc)(er->getArray(),m_createParam,*(this->beginEdit() ) );
					this->endEdit();
				 }
				 break;
			 }
			 case core::componentmodel::topology::POINTSINDICESSWAP:
			 {
				 unsigned int i1 = ( static_cast< const PointsIndicesSwap * >( *changeIt ) )->index[0];
				 unsigned int i2 = ( static_cast< const PointsIndicesSwap* >( *changeIt ) )->index[1];
				 swap( i1, i2 );
				 break;
			 }
			 case core::componentmodel::topology::POINTSADDED:
			 {
				 unsigned int nbPoints = ( static_cast< const PointsAdded * >( *changeIt ) )->getNbAddedVertices();
				 sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( static_cast< const PointsAdded * >( *changeIt ) )->ancestorsList;
				 sofa::helper::vector< sofa::helper::vector< double       > > coefs     = ( static_cast< const PointsAdded * >( *changeIt ) )->coefs;
				 add( nbPoints, ancestors, coefs );
				 break;
			 }
			 case core::componentmodel::topology::POINTSREMOVED:
			 {
				 const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRemoved * >( *changeIt ) )->getArray();
				 remove( tab );
				 break;
			 }
			 case core::componentmodel::topology::POINTSRENUMBERING:
			 {

				 const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRenumbering * >( *changeIt ) )->getIndexArray(); 
				 renumber( tab );
				 break;
			 }
			 default:
				 // Ignore events that are not point related.
				 break;
			}; // switch( changeType )

			++changeIt;
		}
	}

	template <typename T, typename Alloc>
	void PointData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;

		this->endEdit();
	}

	template <typename T, typename Alloc>
	void PointData<T,Alloc>::add( unsigned int nbPoints, 
								const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
								const sofa::helper::vector< sofa::helper::vector< double > >& coefs) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

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



	template <typename T, typename Alloc>
	void PointData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) 
	{
		unsigned int last = this->getValue().size() -1;

		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			m_destroyFunc( index[i], m_destroyParam, data[index[i]] );
			swap( index[i], last );
			--last;
		}

		data.resize( data.size() - index.size() );
		this->endEdit();
	}


	template <typename T, typename Alloc>
	void PointData<T,Alloc>::renumber( const sofa::helper::vector<unsigned int> &index ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		sofa::helper::vector< T,Alloc > copy = this->getValue(); // not very efficient memory-wise, but I can see no better solution...
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
