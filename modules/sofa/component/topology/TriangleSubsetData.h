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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLESUBSETDATA_H
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLESUBSETDATA_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <map>

namespace sofa
{

namespace component
{

namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Triangle Triangle;

	/** \brief Basic creation function for element of type T : simply calls default constructor.
	*
	*/
	template < typename T >
	inline void tsd_basicCreateFunc(unsigned int , 
									void* , T& t, 
									const Triangle& ,  
									const std::vector< unsigned int > &,
									const std::vector< double >&)
	{
		t = T();
		//return;
	}

	/** \brief Basic destruction function for element of type T : does nothing.
	*
	*/
	template < typename T >
	inline void tsd_basicDestroyFunc(unsigned int , void* , T& )
	{
		return;
	}

	/** \brief A class for storing Triangle related data. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: Triangles added, removed, fused, renumbered).
	*/
	template< class T > 
	class TriangleSubsetData : public std::map<unsigned int,T> 
	{
	public:
		/// size_type
		typedef typename std::map<unsigned int,T>::size_type size_type;
		/// reference to a value (read-write)
		typedef typename std::map<unsigned int,T>::reference reference;
		/// const reference to a value (read only)
		typedef typename std::map<unsigned int,T>::const_reference const_reference;

	public:
		/// Constructor
		TriangleSubsetData(size_type n, const T& value): std::map<unsigned int,T>(n,value) {}
		/// Constructor
		TriangleSubsetData(int n, const T& value): std::map<unsigned int,T>(n,value) {}
		/// Constructor
		TriangleSubsetData(long n, const T& value): std::map<unsigned int,T>(n,value) {}
		/// Constructor
		explicit TriangleSubsetData(size_type n): std::map<unsigned int,T>(n) {}
		/// Constructor
		TriangleSubsetData(const std::map<unsigned int,T>& x): std::map<unsigned int,T>(x) {}

#ifdef __STL_MEMBER_TEMPLATES
		/// Constructor
		template <class InputIterator>
		TriangleSubsetData(InputIterator first, InputIterator last): std::map<unsigned int,T>(first,last){}
#else /* __STL_MEMBER_TEMPLATES */
		/// Constructor
		TriangleSubsetData(typename TriangleSubsetData<T>::const_iterator first, typename TriangleSubsetData<T>::const_iterator last): std::map<unsigned int,T>(first,last){}
#endif /* __STL_MEMBER_TEMPLATES */

		/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		TriangleSubsetData( void (*createFunc) (unsigned int, void*, T&, const Triangle &,const std::vector< unsigned int >&, const std::vector< double >&) = tsd_basicCreateFunc,  
			void* createParam  = (void*)NULL, 
			void (*destroyFunc)(unsigned int, void*, T&                                     ) = tsd_basicDestroyFunc, 
			void* destroyParam = (void*)NULL ) 
			: std::map<unsigned int,T>(), 
			m_createFunc(createFunc), m_destroyFunc(destroyFunc), 
			m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
			m_createParam(createParam), m_destroyParam(destroyParam),lastTriangleIndex(0)
		{}

		/// Handle TriangleSetTopology related events, ignore others.
		void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt,
								std::list< const core::topology::TopologyChange *>::const_iterator &end,
								const unsigned int totalTriangleSetArraySize);

		void setCreateFunction(void (*createFunc )(unsigned int, void*, T&, const Triangle&, const std::vector< unsigned int >&, const std::vector< double>& )) 
		{
			m_createFunc=createFunc;
		}

		void setDestroyFunction( void (*destroyFunc)(unsigned int, void*, T&) ) 
		{
			m_destroyFunc=destroyFunc;
		}

		void setDestroyParameter( void* destroyParam ) 
		{
			m_destroyParam=destroyParam;
		}

		void setCreateParameter( void* createParam ) 
		{
			m_createParam=createParam;
		}

		void setCreateTetrahedronFunction(	void (*createTetrahedronFunc)(const std::vector<unsigned int> &, void*,  std::map<unsigned int, T >&) ) 
		{
			m_createTetrahedronFunc=createTetrahedronFunc;
		}

		void setDestroyTetrahedronFunction(void (*destroyTetrahedronFunc)(const std::vector<unsigned int> &, void*,  std::map<unsigned int, T > &) ) 
		{
			m_destroyTetrahedronFunc=destroyTetrahedronFunc;
		}

	private:
		/// Add some values. Values are added at the end of the vector.
		void add( unsigned int nbTriangles,
				const helper::vector< Triangle >& triangle,
				const helper::vector< helper::vector< unsigned int > > &ancestors, 
				const helper::vector< helper::vector< double > >& coefs);

		/// Remove the values corresponding to the Triangles removed.
		void remove( const helper::vector<unsigned int> &index );

		void setTotalTriangleSetArraySize(const unsigned int s) 
		{
			lastTriangleIndex=s-1;
		}

	private:
		/// Creation function, called when adding elements.
		void (*m_createFunc)(unsigned int, void*, T&, const Triangle&, const std::vector< unsigned int >&, const std::vector< double >& );

		/// Destruction function, called when deleting elements.
		void (*m_destroyFunc)(unsigned int, void*, T&);

		/// Creation function, called when adding tetrahedra elements.
		void (*m_createTetrahedronFunc)(const std::vector<unsigned int> &, void*,  std::map<unsigned int, T >&);

		/// Destruction function, called when removing tetrahedra elements.
		void (*m_destroyTetrahedronFunc)(const std::vector<unsigned int> &, void*,  std::map<unsigned int, T > &);

		/** Parameter to be passed to creation function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of TriangleSubsetData.
		*/
		void* m_createParam;

		/** Parameter to be passed to destruction function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of TriangleSubsetData.
		*/
		void* m_destroyParam;

		/// to handle properly the removal of items, the container must know the index of the last element
		unsigned int lastTriangleIndex;
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _TriangleSubsetDATA_H_
