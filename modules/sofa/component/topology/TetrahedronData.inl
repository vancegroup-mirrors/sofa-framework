/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONDATA_INL

#include <sofa/component/topology/TetrahedronData.h>
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

	template <typename T, typename Alloc>
	void TetrahedronData<T,Alloc>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, 
														std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
	{
		while( changeIt != end )
		{
			core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
			case core::componentmodel::topology::TETRAHEDRAADDED:
			{
				const TetrahedraAdded *ta = static_cast< const TetrahedraAdded * >( *changeIt );
				add( ta->getNbAddedTetrahedra(), ta->tetrahedronArray, ta->ancestorsList, ta->coefs );
				break;
			}
			case core::componentmodel::topology::TETRAHEDRAREMOVED:
			{
				const sofa::helper::vector<unsigned int> &tab = ( static_cast< const TetrahedraRemoved *>( *changeIt ) )->getArray();
				remove( tab );
				break;
			}
			default:
			// Ignore events that are not Tetrahedron or Point related.
			break;
			}; 

			++changeIt;
		} 
	}

	template <typename T, typename Alloc>
	void TetrahedronData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		T tmp = (*this)[i1];
		(*this)[i1] = (*this)[i2];
		(*this)[i2] = tmp;
	}

	template <typename T, typename Alloc>
	void TetrahedronData<T,Alloc>::add( unsigned int nbTetrahedra, 
										const sofa::helper::vector< Tetrahedron >& tetrahedron, 
										const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
										const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		// Using default values
		unsigned int i0 = this->size();
		this->resize(i0+nbTetrahedra);
		for (unsigned int i = 0; i < nbTetrahedra; ++i)
		{
			T& t = (*this)[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc(i0+i, m_createParam, t, tetrahedron[i], empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc(i0+i, m_createParam, t, tetrahedron[i], ancestors[i], coefs[i] );
		}
	}



	template <typename T, typename Alloc>
	void TetrahedronData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) {
		unsigned int last = this->size() -1;

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			m_destroyFunc( index[i], m_destroyParam, (*this)[index[i]] );
			swap( index[i], last );
			--last;
		}

		resize( this->size() - index.size() );
	}



} // namespace topology

} // namespace component

} // namespace sofa

#endif // _TetrahedronDATA_INL_
