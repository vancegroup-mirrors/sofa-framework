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
#ifndef SOFA_COMPONENT_TOPOLOGY_QUADDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_QUADDATA_INL

#include <sofa/component/topology/QuadData.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>

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
	void QuadData<T,Alloc>::handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
												std::list< const core::topology::TopologyChange *>::const_iterator &end ) 
	{
		while( changeIt != end )
		{
			core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
			/*
			case core::topology::HEXAHEDRAADDED:
			{
				if (m_createHexahedronFunc) 
				{
					const HexahedraAdded *ea=static_cast< const HexahedraAdded* >( *changeIt );
					(*m_createHexahedronFunc)(ea->hexahedronIndexArray,m_createParam,*this);
				}
				break;
			}
			case core::topology::HEXAHEDRAREMOVED:
			{
				if (m_destroyHexahedronFunc) 
				{
					const HexahedraRemoved *er=static_cast< const HexahedraRemoved * >( *changeIt );
					(*m_destroyHexahedronFunc)(er->getArray(),m_createParam,*this);
				}
				break;
			}
			*/
			case core::topology::QUADSADDED:
			{
				const QuadsAdded *qa=static_cast< const QuadsAdded * >( *changeIt );
				add( qa->getNbAddedQuads(), qa->quadArray, qa->ancestorsList, qa->coefs );
				break;
			}

			case core::topology::QUADSREMOVED:
			{
				const sofa::helper::vector<unsigned int> &tab = ( static_cast< const QuadsRemoved *>( *changeIt ) )->getArray();
				remove( tab );
				break;
			}

			default:
				// Ignore events that are not Quad or Point related.
				break;
			}; // switch( changeType )

			++changeIt;
		}
	}

	template <typename T, typename Alloc>
	void QuadData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void QuadData<T,Alloc>::add( unsigned int nbQuads, 
									const sofa::helper::vector< Quad >& quad, 
									const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
									const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		// Using default values
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		unsigned int i0 = data.size();
		this->resize(i0+nbQuads);

		for (unsigned int i = 0; i < nbQuads; ++i)
		{
			T& t = data[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc( i0+i, m_createParam, t, quad[i], empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc( i0+i, m_createParam, t, quad[i], ancestors[i], coefs[i] );
		}
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void QuadData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		unsigned int last = data.size() -1;

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			m_destroyFunc( index[i], m_destroyParam, data[index[i]] );
			swap( index[i], last );
			--last;
		}

		data.resize( data.size() - index.size() );
		this->endEdit();
	}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _QuadDATA_INL_
