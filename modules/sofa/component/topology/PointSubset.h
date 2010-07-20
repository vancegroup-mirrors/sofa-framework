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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTSUBSET_H
#define SOFA_COMPONENT_TOPOLOGY_POINTSUBSET_H

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/component/component.h>
#include <list>
#include <iostream>

namespace sofa
{
	namespace core{ namespace topology{ class TopologyChange;} }

namespace component
{

namespace topology
{

	/** \brief Basic test function for a new element in the point subset : do not accept any new point by returning false
	*
	*/
	inline bool ps_testNewPointFunc(int , void* , 
									const sofa::helper::vector< unsigned int > &,
									const sofa::helper::vector< double       > &)
	{
		return false;
	}

	/** \brief Basic removal function for a point in the array  : do nothing.
	*
	*/
	inline void ps_removeFunc(int , void* )
	{
		return;
	}

	/** \brief A class for storing indices of points as to define a subset of the DOFs. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: points added, removed, fused, renumbered).
	*/
	class SOFA_COMPONENT_TOPOLOGY_API PointSubset 
	{
	public:
		// forwarding common vector methods and typedefs
		typedef  helper::vector<unsigned int>::value_type                value_type;
		typedef  helper::vector<unsigned int>::pointer                   pointer;
		typedef  helper::vector<unsigned int>::reference                 reference;
		typedef  helper::vector<unsigned int>::const_reference           const_reference;
		typedef  helper::vector<unsigned int>::size_type                 size_type;
		typedef  helper::vector<unsigned int>::difference_type          difference_type;
		typedef  helper::vector<unsigned int>::iterator                  iterator;
		typedef  helper::vector<unsigned int>::const_iterator            const_iterator;
		typedef  helper::vector<unsigned int>::reverse_iterator          reverse_iterator;
		typedef  helper::vector<unsigned int>::const_reverse_iterator    const_reverse_iterator;

	public:
		/// Optionally takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		PointSubset( bool (*testNewPointFunc)(int, void*, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >&)=ps_testNewPointFunc, 
					void* testParam  = (void*)NULL, 
					void (*removeFunc)(int, void*) = ps_removeFunc, 
					void* removeParam = (void*)NULL ) 
		: m_testNewPointFunc(testNewPointFunc), 
		m_removalFunc(removeFunc), m_testParam(testParam), m_removeParam(removeParam), lastPointIndex(0)
		{}

		/// Handle PointSetTopology related events, ignore others.
		void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
								std::list< const core::topology::TopologyChange *>::const_iterator &end,
								const unsigned int totalPointSetArraySize);

		// defining operators so that pointSubset can be used in a Data (see Data class).
		friend SOFA_COMPONENT_TOPOLOGY_API std::ostream& operator<< (std::ostream& ostream, const PointSubset& pointSubset);
		friend SOFA_COMPONENT_TOPOLOGY_API std::istream& operator>> (std::istream& i,             PointSubset& pointSubset);

		void setTestFunction(bool (*testNewPointFunc )(int, void*, const sofa::helper::vector< unsigned int > &, 
														const sofa::helper::vector< double >& )) 
		{
			m_testNewPointFunc=testNewPointFunc;
		}

		void setRemovalFunction( void (*removeFunc)(int, void*) ) 
		{
			m_removalFunc=removeFunc;
		}

		void setRemovalParameter( void* removeParam ) 
		{
			m_removeParam=removeParam;
		}

		void setTestParameter( void* testParam ) 
		{
			m_testParam=testParam;
		}

		const helper::vector< unsigned int > & getArray() const { return m_subset; }

		iterator begin() { return m_subset.begin(); }

		iterator end() {return m_subset.end(); }

		const_iterator begin() const { return m_subset.begin(); }

		const_iterator end() const { return m_subset.end(); }

		reverse_iterator rbegin() { return m_subset.rbegin(); }

		reverse_iterator rend() { return m_subset.rend(); }

		const_reverse_iterator rbegin() const { return m_subset.rbegin(); }

		const_reverse_iterator rend() const { return m_subset.rend(); }

		size_type size() const { return m_subset.size(); }

		size_type max_size() const { return m_subset.max_size(); }

		size_type capacity() const { return m_subset.capacity(); }

		bool empty() const { return m_subset.empty(); }

		reference operator[]( size_type n ) { return m_subset[n]; }

		const_reference operator[]( size_type i ) const { return m_subset[i]; }

		//        PointSubset& operator=( const sofa::helper::vector<unsigned int> & vector ) { m_subset( vector ); return *this; }

		//        PointSubset& operator=( const PointSubset& pointSubset ) { m_subset( pointSubset.m_subset ); return *this; }

		void reserve( size_t n ) { m_subset.reserve( n ); }

		reference front() { return m_subset.front(); }

		const_reference front() const { return m_subset.front(); }

		reference back() { return m_subset.back(); }

		const_reference back() const { return m_subset.back(); }

		void push_back( const unsigned int  t ) { m_subset.push_back( t ); }

		void pop_back() { m_subset.pop_back(); }

		void swap( helper::vector<unsigned int> &vector ) { m_subset.swap( vector ); }

		void swap( PointSubset &pointSubset) { m_subset.swap( pointSubset.m_subset ); }

		void clear() { m_subset.clear(); }

		void resize( size_t n ) { m_subset.resize( n ); }

	private:
		/// Swaps values at indices i1 and i2.
		void swap( unsigned int i1, unsigned int i2 );

		/// Add some values. Values are added at the end of the vector.
		void add( unsigned int nbPoints, 
				const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
				const sofa::helper::vector< sofa::helper::vector< double       > >& coefs);

		/// Remove the values corresponding to the points removed.
		void remove( const sofa::helper::vector<unsigned int> &index );

		/// Reorder the values.
		void renumber( const sofa::helper::vector<unsigned int> &index );

		void setTotalPointSetArraySize(const unsigned int s) { lastPointIndex=s-1; }

	private:
		/// Actual point subset stored.
		helper::vector< unsigned int > m_subset;

		/// test function, called when new points are created.
		bool (*m_testNewPointFunc)(int, void*, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >&);

		/// Removal function, called when points in the subset are removed 
		void (*m_removalFunc)(int, void*);

		/** Parameter to be passed to test function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of pointSubset.
		*/
		void* m_testParam;

		/** Parameter to be passed to removal function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of pointSubset.
		*/
		void* m_removeParam;

		/// to handle properly the removal of items, the container must know the index of the last element
		unsigned int lastPointIndex;
	};

	/// Needed to be compliant with Datas.
	SOFA_COMPONENT_TOPOLOGY_API std::ostream& operator<< (std::ostream& os, const PointSubset& pointSubset);

	/// Needed to be compliant with Datas.
	SOFA_COMPONENT_TOPOLOGY_API std::istream& operator>>(std::istream& i, PointSubset& pointSubset);

} // namespace topology

} // namespace component

// Specialization of the defaulttype::DataTypeInfo type traits template

namespace defaulttype
{

template<>
struct DataTypeInfo< sofa::component::topology::PointSubset > : public VectorTypeInfo<sofa::component::topology::PointSubset >
{
    static const char* name() { return "PointSubset"; }
};

} // namespace defaulttype

} // namespace sofa

#endif // _POINTSUBSET_H_
