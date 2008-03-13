#ifndef SOFA_COMPONENT_TOPOLOGY_QUADDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_QUADDATA_INL

#include <sofa/component/topology/QuadSetTopology.h>
//#include <sofa/component/topology/HexahedronSetTopology.h>
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
	void QuadData<T,Alloc>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
	{
		while( changeIt != end )
		{
			core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			// Since we are using identifier, we can safely use C type casts.

			switch( changeType ) {
				/*
				case core::componentmodel::topology::HEXAHEDRAADDED:
					{
						if (m_createHexahedronFunc) {
							const HexahedraAdded *ea=dynamic_cast< const HexahedraAdded* >( *changeIt );
							(*m_createHexahedronFunc)(ea->hexahedronIndexArray,m_createParam,*this);
						}
						break;
					}
				case core::componentmodel::topology::HEXAHEDRAREMOVED:
					{
						if (m_destroyHexahedronFunc) {
							const HexahedraRemoved *er=dynamic_cast< const HexahedraRemoved * >( *changeIt );
							(*m_destroyHexahedronFunc)(er->getArray(),m_createParam,*this);
						}
						break;
					}
					*/
				case core::componentmodel::topology::QUADSADDED:
					{
						const QuadsAdded *qa=dynamic_cast< const QuadsAdded * >( *changeIt );
						add( ta->getNbAddedQuads(), qa->quadArray, qa->ancestorsList, qa->coefs );
						break;
					}

				case core::componentmodel::topology::QUADSREMOVED:
					{
						const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const QuadsRemoved *>( *changeIt ) )->getArray();
						remove( tab );
						break;
					}

				default:
					// Ignore events that are not Quad or Point related.
					break;
			}; // switch( changeType )

			++changeIt;
		} // while( changeIt != last; )
	}



		template <typename T, typename Alloc>
		void QuadData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) {
			T tmp = (*this)[i1];
			(*this)[i1] = (*this)[i2];
			(*this)[i2] = tmp;
		}
        template <typename T, typename Alloc>
        void QuadData<T,Alloc>::add( unsigned int nbQuads, const sofa::helper::vector< Quad >& quad, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) {
            // Using default values
    
            for (unsigned int i = 0; i < nbQuads; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
                m_createFunc( this->size() + i, m_createParam, t, quad[i], (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
                m_createFunc( this->size() + i, m_createParam, t, quad[i], ancestors[i], coefs[i] );
                push_back( t );
            }
        }



        template <typename T, typename Alloc>
        void QuadData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) {
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

#endif // _QuadDATA_INL_
