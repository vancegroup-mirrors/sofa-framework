#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_INL

#include <sofa/component/topology/TriangleSetTopology.h>
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
	void TriangleData<T,Alloc>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end ) 
	{
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
						const TrianglesAdded *ta=dynamic_cast< const TrianglesAdded * >( *changeIt );
						add( ta->getNbAddedTriangles(), ta->triangleArray, ta->ancestorsList, ta->coefs );
						break;
					}

				case core::componentmodel::topology::TRIANGLESREMOVED:
					{
						const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const TrianglesRemoved *>( *changeIt ) )->getArray();
						remove( tab );
						break;
					}

				default:
					// Ignore events that are not Triangle or Point related.
					break;
			}; // switch( changeType )

			++changeIt;
		} // while( changeIt != last; )
	}



		template <typename T, typename Alloc>
		void TriangleData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) {
			T tmp = (*this)[i1];
			(*this)[i1] = (*this)[i2];
			(*this)[i2] = tmp;
		}
        template <typename T, typename Alloc>
        void TriangleData<T,Alloc>::add( unsigned int nbTriangles, const sofa::helper::vector< Triangle >& triangle, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) {
            // Using default values
    
            for (unsigned int i = 0; i < nbTriangles; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
                m_createFunc( this->size() + i, m_createParam, t, triangle[i], (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
                m_createFunc( this->size() + i, m_createParam, t, triangle[i], ancestors[i], coefs[i] );
                push_back( t );
            }
        }



        template <typename T, typename Alloc>
        void TriangleData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) {
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

#endif // _TriangleDATA_INL_
