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
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_INL

#include <sofa/component/topology/HexahedronData.h>
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
	void HexahedronData<T,Alloc>::handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
														std::list< const core::topology::TopologyChange *>::const_iterator &end ) 
	{
		while( changeIt != end )
		{
			core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
			case core::topology::HEXAHEDRAADDED:
			{
				const HexahedraAdded *ta=static_cast< const HexahedraAdded * >( *changeIt );
				add( ta->getNbAddedHexahedra(), ta->hexahedronArray, ta->ancestorsList, ta->coefs );
				break;
			}
			case core::topology::HEXAHEDRAREMOVED:
			{
				const sofa::helper::vector<unsigned int> &tab = ( static_cast< const HexahedraRemoved *>( *changeIt ) )->getArray();
				remove( tab );
				break;
			}
			default:
				// Ignore events that are not Hexahedron or Point related.
				break;
			}; // switch( changeType )

			++changeIt;
		} // while( changeIt != last; )
	}


	template <typename T, typename Alloc>
	void HexahedronData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;

		this->endEdit();
	}

	template <typename T, typename Alloc>
	void HexahedronData<T,Alloc>::add( unsigned int nbHexahedra, 
										const sofa::helper::vector< Hexahedron >& hexahedron, 
										const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
										const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		// Using default values
		unsigned int i0 = data.size();
		data.resize(i0+nbHexahedra);
		for (unsigned int i = 0; i < nbHexahedra; ++i)
		{
			T& t = data[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc(i0+i, m_createParam, t, hexahedron[i], empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc(i0+i, m_createParam, t, hexahedron[i], ancestors[i], coefs[i] );
		}
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void HexahedronData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) 
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

#endif // _HexahedronDATA_INL_
