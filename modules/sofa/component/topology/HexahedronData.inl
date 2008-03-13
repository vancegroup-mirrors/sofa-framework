#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_INL

#include <sofa/component/topology/HexahedronSetTopology.h>
#include <sofa/helper/vector.h>
#include <vector>
#include <iostream>


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
        void HexahedronData<T,Alloc>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
        {
			while( changeIt != end )
			{
				core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

				// Since we are using identifier, we can safely use C type casts.

				switch( changeType ) {
				case core::componentmodel::topology::HEXAHEDRAADDED:
					{
						const HexahedraAdded *ta=dynamic_cast< const HexahedraAdded * >( *changeIt );
						add( ta->getNbAddedHexahedra(), ta->triangleArray, ta->ancestorsList, ta->coefs );
						break;
					}

                    case core::componentmodel::topology::HEXAHEDRAREMOVED:
                        {
                            const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const HexahedraRemoved *>( *changeIt ) )->getArray();
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
        void HexahedronData<T,Alloc>::add( unsigned int nbHexahedra, const sofa::helper::vector< Hexahedron >& triangle, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) {
            // Using default values
            unsigned int size = m_data.size();
            for (unsigned int i = 0; i < nbHexahedra; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
                m_createFunc( size + i, m_createParam, t, triangle[i], (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
                m_createFunc( size + i, m_createParam, t, triangle[i], ancestors[i], coefs[i] );
                m_data.push_back( t );
            }
        }



        template <typename T, typename Alloc>
        void HexahedronData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) {
            unsigned int last = m_data.size() -1;

            for (unsigned int i = 0; i < index.size(); ++i)
            {
                swap( index[i], last );
                m_destroyFunc( index[i], m_destroyParam, (*this)[last] );
                --last;
            }

            m_data.resize( m_data.size() - index.size() );
        }



    } // namespace topology

} // namespace component

} // namespace sofa

#endif // _HexahedronDATA_INL_
