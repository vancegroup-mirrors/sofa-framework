/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_POINTDATA_INL

#include <sofa/component/topology/PointData.h>
#include <sofa/component/topology/EdgeSetTopology.h>
#include <sofa/component/topology/TriangleSetTopology.h>
#include <sofa/component/topology/TetrahedronSetTopology.h>


namespace sofa
{

namespace component
{

namespace topology
{

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ////////////////////////////////////////implementation//////////////////////////////////////////////////////
        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////



        template <typename T, typename Alloc>
        void PointData<T,Alloc>::handleTopologyEvents( std::list<  const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) {
            while( changeIt != end )
            {
                core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

                // Since we are using identifier, we can safely use C type casts.

                switch( changeType ) {
					 case core::componentmodel::topology::TETRAHEDRAADDED:
                        {
							if (m_createTetrahedronFunc) {
								const TetrahedraAdded *ea=dynamic_cast< const TetrahedraAdded* >( *changeIt );
								(*m_createTetrahedronFunc)(ea->tetrahedronIndexArray,m_createParam,*this);
							}
							break;
                        }
					case core::componentmodel::topology::TETRAHEDRAREMOVED:
                        {
							if (m_destroyTetrahedronFunc) {
								const TetrahedraRemoved *er=dynamic_cast< const TetrahedraRemoved * >( *changeIt );
								(*m_destroyTetrahedronFunc)(er->getArray(),m_createParam,*this);
							}
							break;
                        }		
					 case core::componentmodel::topology::TRIANGLESADDED:
                        {
							if (m_createTriangleFunc) {
								const TrianglesAdded *ea=dynamic_cast< const TrianglesAdded* >( *changeIt );
								(*m_createTriangleFunc)(ea->triangleIndexArray,m_createParam,*this);
							}
							break;
                        }
					case core::componentmodel::topology::TRIANGLESREMOVED:
                        {
							if (m_destroyTriangleFunc) {
								const TrianglesRemoved *er=dynamic_cast< const TrianglesRemoved * >( *changeIt );
								(*m_destroyTriangleFunc)(er->getArray(),m_createParam,*this);
							}
							break;
                        }						 
					case core::componentmodel::topology::EDGESADDED:
                        {
							if (m_createEdgeFunc) {
								const EdgesAdded *ea=dynamic_cast< const EdgesAdded* >( *changeIt );
								(*m_createEdgeFunc)(ea->edgeIndexArray,m_createParam,*this);
							}
							break;
                        }
					case core::componentmodel::topology::EDGESREMOVED:
                        {
							if (m_destroyEdgeFunc) {
								const EdgesRemoved *er=dynamic_cast< const EdgesRemoved * >( *changeIt );
								(*m_destroyEdgeFunc)(er->getArray(),m_createParam,*this);
							}
							break;
                        }

                    case core::componentmodel::topology::POINTSINDICESSWAP:
                        {
                            unsigned int i1 = ( dynamic_cast< const PointsIndicesSwap * >( *changeIt ) )->index[0];
                            unsigned int i2 = ( dynamic_cast< const PointsIndicesSwap* >( *changeIt ) )->index[1];
                            swap( i1, i2 );
                            break;
                        }

                    case core::componentmodel::topology::POINTSADDED:
                        {
                            unsigned int nbPoints = ( dynamic_cast< const PointsAdded * >( *changeIt ) )->getNbAddedVertices();
                            sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( dynamic_cast< const PointsAdded * >( *changeIt ) )->ancestorsList;
                            sofa::helper::vector< sofa::helper::vector< double       > > coefs     = ( dynamic_cast< const PointsAdded * >( *changeIt ) )->coefs;
                            add( nbPoints, ancestors, coefs );
                            break;
                        }

                    case core::componentmodel::topology::POINTSREMOVED:
                        {
                            const sofa::helper::vector<unsigned int> tab = ( dynamic_cast< const PointsRemoved * >( *changeIt ) )->getArray();
                            remove( tab );
                            break;
                        }

                    case core::componentmodel::topology::POINTSRENUMBERING:
                        {
                            const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const PointsRenumbering * >( *changeIt ) )->getIndexArray();
                            renumber( tab );
                            break;
                        }

                    default:
                        // Ignore events that are not point related.
                        break;
                }; // switch( changeType )

                ++changeIt;
            } // while( changeIt != last; )
        }



        template <typename T, typename Alloc>
        void PointData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) {
            T tmp = (*this)[i1];
            (*this)[i1] = (*this)[i2];
            (*this)[i2] = tmp;
        }



        template <typename T, typename Alloc>
        void PointData<T,Alloc>::add( unsigned int nbPoints, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs) {
            // Using default values
            unsigned int _size = this->size();
            for (unsigned int i = 0; i < nbPoints; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
                m_createFunc( _size + i, m_createParam, t, (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
                m_createFunc( _size + i, m_createParam, t, ancestors[i], coefs[i] );
                push_back( t );
            }
        }



        template <typename T, typename Alloc>
        void PointData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) {
            unsigned int last = this->size() -1;

            for (unsigned int i = 0; i < index.size(); ++i)
            {
                swap( index[i], last );
                m_destroyFunc( index[i], m_destroyParam, (*this)[last] );
                --last;
            }

            resize( this->size() - index.size() );
        }


        template <typename T, typename Alloc>
        void PointData<T,Alloc>::renumber( const sofa::helper::vector<unsigned int> &index ) {
            sofa::helper::vector< T,Alloc > copy = (*this); // not very efficient memory-wise, but I can see no better solution...
            for (unsigned int i = 0; i < index.size(); ++i)
            {
                (*this)[i] = copy[ index[i] ];
            }
        }


    } // namespace topology

} // namespace component

} // namespace sofa

#endif // _POINTDATA_INL_
