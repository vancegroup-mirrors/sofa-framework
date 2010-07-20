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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_H
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/vector.h>

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
	inline void trd_basicCreateFunc(int , 
									void* , T& t, 
									const Triangle& ,  
									const sofa::helper::vector< unsigned int > &,
									const sofa::helper::vector< double >&)
	{
		t = T();
		//return;
	}

	/** \brief Basic destruction function for element of type T : does nothing.
	*
	*/
	template < typename T >
	inline void trd_basicDestroyFunc(int , void* , T& )
	{
		return;
	}

	/** \brief A class for storing Triangle related data. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: Triangles added, removed, fused, renumbered).
	*/
	template< class T, class Alloc = helper::CPUMemoryManager<T> >
	class TriangleData : public sofa::core::objectmodel::Data<sofa::helper::vector<T, Alloc> >
	{
	public:
		/// size_type
		typedef typename sofa::helper::vector<T,Alloc>::size_type size_type;
		/// reference to a value (read-write)
		typedef typename sofa::helper::vector<T,Alloc>::reference reference;
		/// const reference to a value (read only)
		typedef typename sofa::helper::vector<T,Alloc>::const_reference const_reference;
		/// const iterator 
		typedef typename sofa::helper::vector<T,Alloc>::const_iterator const_iterator;

	public:
		/// Constructor
            TriangleData( const typename sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >::InitData& data,
			  void (*createFunc) (int, void*, T&, const Triangle &,const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = trd_basicCreateFunc,
				void* createParam  = (void*)NULL,
				void (*destroyFunc)(int, void*, T&) = trd_basicDestroyFunc,
				void* destroyParam = (void*)NULL )
		: sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(data), 
		m_createFunc(createFunc), m_destroyFunc(destroyFunc), 
		m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
		m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Constructor
		TriangleData(size_type n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		TriangleData(int n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		TriangleData(long n, const T& value): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		explicit TriangleData(size_type n): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n);
			this->endEdit();
		}
		/// Constructor
		TriangleData(const sofa::helper::vector<T, Alloc>& x): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			(*data) = x;
			this->endEdit();
		}

#ifdef __STL_MEMBER_TEMPLATES
		/// Constructor
		template <class InputIterator>
		TriangleData(InputIterator first, InputIterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#else /* __STL_MEMBER_TEMPLATES */
		/// Constructor
		TriangleData(const_iterator first, const_iterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#endif /* __STL_MEMBER_TEMPLATES */

		/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		TriangleData( void (*createFunc) (int, void*, T&, const Triangle& ,const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = trd_basicCreateFunc,  
					void* createParam  = (void*)NULL, 
					void (*destroyFunc)(int, void*, T&                                     ) = trd_basicDestroyFunc, 
					void* destroyParam = (void*)NULL ) 
		: sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false),
		m_createFunc(createFunc), m_destroyFunc(destroyFunc),
		m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
		m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Handle TriangleSetTopology related events, ignore others.
		void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
								std::list< const core::topology::TopologyChange *>::const_iterator &end );

		void setCreateFunction(void (*createFunc )(int, void*, T&, const Triangle&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double>& )) 
		{
			m_createFunc=createFunc;
		}

		void setDestroyFunction( void (*destroyFunc)(int, void*, T&) ) 
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

		void setCreateTetrahedronFunction(	void (*createTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &) ) 
		{
			m_createTetrahedronFunc=createTetrahedronFunc;
		}

		void setDestroyTetrahedronFunction(void (*destroyTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &) ) 
		{
			m_destroyTetrahedronFunc=destroyTetrahedronFunc;
		}

		T& operator[](int i)
    		{
			sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
			T& result = data[i];
			this->endEdit();
        		return result;
    		}


	private:
		/// Swaps values at indices i1 and i2.
		void swap( unsigned int i1, unsigned int i2 );

		/// Add some values. Values are added at the end of the vector.
		void add( unsigned int nbTriangles,
				const sofa::helper::vector< Triangle >& triangle,
				const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors, 
				const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

		/// Remove the values corresponding to the Triangles removed.
		void remove( const sofa::helper::vector<unsigned int> &index );

	private:
		/// Creation function, called when adding elements.
		void (*m_createFunc)(int, void*, T&, const Triangle&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >& );

		/// Destruction function, called when deleting elements.
		void (*m_destroyFunc)(int, void*, T&);

		/// Creation function, called when adding tetrahedra elements.
		void (*m_createTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &);

		/// Destruction function, called when removing tetrahedra elements.
		void (*m_destroyTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &);

		/** Parameter to be passed to creation function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of TriangleData.
		*/
		void* m_createParam;

		/** Parameter to be passed to destruction function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of TriangleData.
		*/
		void* m_destroyParam;
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _TriangleDATA_H_
