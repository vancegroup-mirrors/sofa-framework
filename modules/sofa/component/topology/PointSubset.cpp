#include <sofa/component/topology/PointSubset.h>
#include <sofa/component/topology/PointSetTopology.h> 


namespace sofa
{

namespace component
{

namespace topology
{

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////implementation//////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void PointSubset::handleTopologyEvents( std::list<  const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, 
		std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end,
		const unsigned int totalPointSetArraySize) {
			setTotalPointSetArraySize(totalPointSetArraySize);
			while( changeIt != end )
			{
				core::componentmodel::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

				// Since we are using identifier, we can safely use C type casts.

				switch( changeType ) {

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
							helper::vector< helper::vector< unsigned int > > ancestors = ( dynamic_cast< const PointsAdded * >( *changeIt ) )->ancestorsList;
							helper::vector< helper::vector< double       > > coefs     = ( dynamic_cast< const PointsAdded * >( *changeIt ) )->coefs;
							add( nbPoints, ancestors, coefs);
							lastPointIndex+=nbPoints;
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


	void PointSubset::swap( unsigned int i1, unsigned int i2 ) {
		helper::vector<unsigned int>::iterator it= find(m_subset.begin(),m_subset.end(),i1);
		if (it!=m_subset.end()) 
			(*it)=i2;
	}



	void PointSubset::add( unsigned int nbPoints, const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, const sofa::helper::vector< sofa::helper::vector< double > >& coefs) {
		// Using default values
		unsigned int size = m_subset.size();
		bool test;
		for (unsigned int i = 0; i < nbPoints; ++i)
		{
			if (ancestors== (const sofa::helper::vector< sofa::helper::vector< unsigned int > >)0)
				test=(*m_testNewPointFunc)( size + i, m_testParam,(const sofa::helper::vector< unsigned int >)0, (const sofa::helper::vector< double  >)0 );
			else
				test=(*m_testNewPointFunc)( size + i, m_testParam, ancestors[i], coefs[i] );
			if (test)
				m_subset.push_back( size+i );
		}
	}

	void PointSubset::remove( const sofa::helper::vector<unsigned int> &index) {

		helper::vector<unsigned int>::iterator it;

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			swap(lastPointIndex,index[i]);
			it= find(m_subset.begin(),m_subset.end(),index[i]);
			if (it!=m_subset.end()) {

				m_removalFunc( index[i], m_removeParam); 

				m_subset.erase(it); // method "erase" also resizes "m_subset"
			}
			--lastPointIndex;
		}

	}

	void PointSubset::renumber( const sofa::helper::vector<unsigned int> &index ) {
		for (unsigned int i = 0; i < m_subset.size(); ++i)
		{
			m_subset[i] =index[m_subset[i] ];
		}
	}
	/// Needed to be compliant with DataFields.
	std::ostream& operator<< (std::ostream& os, const PointSubset& pointSubset)
	{
		return os << pointSubset.m_subset;
	}

	/// Needed to be compliant with DataFields.
	std::istream& operator>>(std::istream& i, PointSubset& pointSubset)
	{
		return i >> pointSubset.m_subset;
	}

    } // namespace topology

} // namespace component

} // namespace sofa

