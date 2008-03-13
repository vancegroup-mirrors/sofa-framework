#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESUBSETDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_EDGESUBSETDATA_INL

#include <sofa/component/topology/EdgeSubsetData.h>
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



       template <typename T>
        void EdgeSubsetData<T>::handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, 
			std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end,
			const unsigned int totalEdgeSetArraySize) 
        {
			setTotalEdgeSetArraySize(totalEdgeSetArraySize);
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
							const EdgesAdded *ea=dynamic_cast< const EdgesAdded * >( *changeIt );
                            add( ea->getNbAddedEdges(), ea->edgeArray, ea->ancestorsList, ea->coefs );
                            break;
                        }

                    case core::componentmodel::topology::EDGESREMOVED:
                        {
                            const std::vector<unsigned int> &tab = ( dynamic_cast< const EdgesRemoved *>( *changeIt ) )->getArray();
                            remove( tab );
                            break;
                        }

                   default:
                        // Ignore events that are not Edge or Point related.
                        break;
                }; // switch( changeType )

                ++changeIt;
            } // while( changeIt != last; )
        }



        template <typename T>
        void EdgeSubsetData<T>::add( unsigned int nbEdges, const sofa::helper::vector< Edge >& edge, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) {
            // Using default values
            unsigned int s = this->size();
            for (unsigned int i = 0; i < nbEdges; ++i)
            {
                T t;
				if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
					m_createFunc( s + i, m_createParam, t, edge[i], (const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
				else
					m_createFunc( s + i, m_createParam, t, edge[i], ancestors[i], coefs[i] );
				// test that there are no item with the key = s+i
				assert(this->find(s+i)==this->end());
				insert(std::pair<unsigned int,T>(s+i,t));
            }
			lastEdgeIndex+=nbEdges;
        }



        template <typename T>
        void EdgeSubsetData<T>::remove( const std::vector<unsigned int> &index ) {
			typename std::map< unsigned int, T >::iterator it,itend;

            for (unsigned int i = 0; i < index.size(); ++i)
            {
				it=this->find(index[i]);
				if (it!=this->end()) {
					m_destroyFunc( index[i], m_destroyParam, (*this)[index[i]] );
					/// the edge lastEdgeIndex is now set to index index[i]
					itend=this->find(lastEdgeIndex);
					if (itend!=this->end()) {
						/// change the key of the item associated with key lastEdgeIndex 
						T tmp=(*itend).second;
						erase(itend);
						insert(std::pair<unsigned int,T>(index[i],tmp));
					}
					erase(it);
				}
				--lastEdgeIndex;
			}
            std::cout << "EdgeSubsetData: vector has now "<<this->size()<<" entries."<<std::endl;
        }



    } // namespace topology

} // namespace component

} // namespace sofa

#endif // _EdgeDATA_INL_
