#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONDATA_INL

#include <sofa/component/topology/TetrahedronSetTopology.h>
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
        void TetrahedronData<T,Alloc>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
        {
			while( changeIt != end )
			{
				core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

				// Since we are using identifier, we can safely use C type casts.

				switch( changeType ) {
				case core::componentmodel::topology::TETRAHEDRAADDED:
					{
						const TetrahedraAdded *ta=dynamic_cast< const TetrahedraAdded * >( *changeIt );
						add( ta->getNbAddedTetrahedra(), ta->tetrahedronArray, ta->ancestorsList, ta->coefs );
						break;
					}

                    case core::componentmodel::topology::TETRAHEDRAREMOVED:
                        {
                            const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const TetrahedraRemoved *>( *changeIt ) )->getArray();
                            remove( tab );
                            break;
                        }

                   default:
                        // Ignore events that are not Tetrahedron or Point related.
                        break;
                }; // switch( changeType )

                ++changeIt;
            } // while( changeIt != last; )
        }


		template <typename T, typename Alloc>
		void TetrahedronData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) {
			T tmp = (*this)[i1];
			(*this)[i1] = (*this)[i2];
			(*this)[i2] = tmp;
		}
        template <typename T, typename Alloc>
        void TetrahedronData<T,Alloc>::add( unsigned int nbTetrahedra, const sofa::helper::vector< Tetrahedron >& tetrahedron, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) {
            // Using default values
            for (unsigned int i = 0; i < nbTetrahedra; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
                m_createFunc(this->size() + i, m_createParam, t, tetrahedron[i], (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
                m_createFunc(this->size() + i, m_createParam, t, tetrahedron[i], ancestors[i], coefs[i] );
                push_back( t );
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
