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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTDATA_H
#define SOFA_COMPONENT_TOPOLOGY_POINTDATA_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/vector.h>
#include <sofa/component/component.h>
#include <list>

namespace sofa
{
	namespace core { namespace componentmodel { namespace topology { class TopologyChange; }}}
	
namespace component
{

namespace topology
{

	/** \brief Basic creation function for element of type T : simply calls default constructor.
	*
	*/
	template < typename T >
	inline void pd_basicCreateFunc(int , void* , T& t, 
									const sofa::helper::vector< unsigned int > &,
									const sofa::helper::vector< double       > &)
	{
		t = T();
		//return;
	}

	/** \brief Basic destruction function for element of type T : does nothing.
	*
	*/
	template < typename T >
	inline void pd_basicDestroyFunc(int , void* , T& )
	{
		return;
	}

	/** \brief A class for storing point related data. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: points added, removed, fused, renumbered).
	*/
	template< class T, class VecT = helper::vector<T> > 
	class PointData : public sofa::core::objectmodel::Data< VecT >//public container_type 
	{
	public:
	     typedef T value_type;
	     typedef VecT container_type;
		/// size_type
		typedef typename VecT::size_type size_type;
		/// reference to a value (read-write)
		typedef typename VecT::reference reference;
		/// const reference to a value (read only)
		typedef typename VecT::const_reference const_reference;
		/// const iterator 
		typedef typename VecT::const_iterator const_iterator;
				
		/// Creation function, called when adding elements.
		typedef void (*t_createFunc)(int, void*, T&, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >&);
		/// Destruction function, called when deleting elements.
		typedef void (*t_destroyFunc)(int, void*, T&);
		/// Creation function, called when adding edges elements.
		typedef void (*t_createEdgeFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Destruction function, called when removing edges elements.
		typedef void (*t_destroyEdgeFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Creation function, called when adding triangles elements.
		typedef void (*t_createTriangleFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Destruction function, called when removing triangles elements.
		typedef void (*t_destroyTriangleFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Creation function, called when adding quads elements.
		typedef void (*t_createQuadFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Destruction function, called when removing quads elements.
		typedef void (*t_destroyQuadFunc)(const sofa::helper::vector<unsigned int> &, void*,  container_type &);
		/// Creation function, called when adding tetrahedra elements.
		typedef void (*t_createTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, container_type &);
		/// Destruction function, called when removing tetrahedra elements.
		typedef void (*t_destroyTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, container_type &);
		
	public:
		/// Constructor
            PointData(const typename sofa::core::objectmodel::Data< container_type >::InitData& data,
 			  void (*createFunc) (int, void*, T&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = pd_basicCreateFunc,
			  void* createParam  = (void*)NULL,
			  void (*destroyFunc)(int, void*, T& ) = pd_basicDestroyFunc,
			  void* destroyParam = (void*)NULL )
		: sofa::core::objectmodel::Data< container_type >(data), 
		m_createFunc(createFunc), m_destroyFunc(destroyFunc), 
		m_createEdgeFunc(0), m_destroyEdgeFunc(0),
		m_createTriangleFunc(0), m_destroyTriangleFunc(0), 
		m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
		m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Constructor
		PointData(size_type n, const T& value) : sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		PointData(int n, const T& value) : sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		PointData(long n, const T& value): sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		explicit PointData(size_type n): sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->resize(n);
			this->endEdit();
		}
		/// Constructor
		PointData(const container_type& x): sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			(*data) = x;
			this->endEdit();
		}

#ifdef __STL_MEMBER_TEMPLATES
		/// Constructor
		template <class InputIterator>
		PointData(InputIterator first, InputIterator last): sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#else /* __STL_MEMBER_TEMPLATES */
		/// Constructor
		PointData(const_iterator first, const_iterator last): sofa::core::objectmodel::Data< container_type >(0, false, false)
		{
			container_type* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#endif /* __STL_MEMBER_TEMPLATES */


		/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		PointData( void (*createFunc) (int, void*, T&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = pd_basicCreateFunc,  
					void* createParam  = (void*)NULL, 
					void (*destroyFunc)(int, void*, T&                                     ) = pd_basicDestroyFunc, 
					void* destroyParam = (void*)NULL ) 
		: sofa::core::objectmodel::Data< container_type >(0, false, false), 
		m_createFunc(createFunc), m_destroyFunc(destroyFunc), 
		m_createEdgeFunc(0), m_destroyEdgeFunc(0),
		m_createTriangleFunc(0), m_destroyTriangleFunc(0), 
		m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
		m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Handle PointSetTopology related events, ignore others.
		void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
									std::list< const core::topology::TopologyChange *>::const_iterator &end );

		void setCreateFunction(t_createFunc createFunc) 
		{
			m_createFunc=createFunc;
		}

		void setDestroyFunction(t_destroyFunc destroyFunc) 
		{
			m_destroyFunc=destroyFunc;
		}

		void setCreateEdgeFunction(t_createEdgeFunc createEdgeFunc) 
		{
			m_createEdgeFunc=createEdgeFunc;
		}
		void setDestroyEdgeFunction(t_destroyEdgeFunc destroyEdgeFunc) {
			m_destroyEdgeFunc=destroyEdgeFunc;
		}

		void setCreateTriangleFunction(t_createTriangleFunc createTriangleFunc) 
		{
			m_createTriangleFunc=createTriangleFunc;
		}

		void setDestroyTriangleFunction(t_destroyTriangleFunc destroyTriangleFunc) 
		{
			m_destroyTriangleFunc=destroyTriangleFunc;
		}

		void setCreateQuadFunction(t_createQuadFunc createQuadFunc) 
		{
			m_createQuadFunc=createQuadFunc;
		}

		void setDestroyQuadFunction(t_destroyQuadFunc destroyQuadFunc) 
		{
			m_destroyQuadFunc=destroyQuadFunc;
		}

		void setCreateTetrahedronFunction(t_createTetrahedronFunc createTetrahedronFunc) 
		{
			m_createTetrahedronFunc=createTetrahedronFunc;
		}

		void setDestroyTetrahedronFunction(t_destroyTetrahedronFunc destroyTetrahedronFunc) 
		{
			m_destroyTetrahedronFunc=destroyTetrahedronFunc;
		}

		void setDestroyParameter( void* destroyParam ) 
		{
			m_destroyParam=destroyParam;
		}

		void setCreateParameter( void* createParam ) 
		{
			m_createParam=createParam;
		}

		T& operator[](int i)
    		{
			container_type& data = *(this->beginEdit());
			T& result = data[i];
			this->endEdit();
        		return result;
    		}
#ifdef POINT_DATA_VECTOR_ACCESS 
		size_type size()
		{
			return this->getValue().size();
		}

		void resize(size_type n)
		{
			container_type& data = *(this->beginEdit());
			data.resize(n);
			this->endEdit();
		}

		void push_back(T& elem)
		{
			container_type& data = *(this->beginEdit());
			data.push_back(elem);
			this->endEdit();
		}
#endif /* POINT_DATA_VECTOR_ACCESS */

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

	private:
		t_createFunc m_createFunc;
		t_destroyFunc m_destroyFunc;
		t_createEdgeFunc m_createEdgeFunc;
		t_destroyEdgeFunc m_destroyEdgeFunc;
		t_createTriangleFunc m_createTriangleFunc;
		t_destroyTriangleFunc m_destroyTriangleFunc;
		t_createQuadFunc m_createQuadFunc;
		t_destroyQuadFunc m_destroyQuadFunc;
		t_createTetrahedronFunc m_createTetrahedronFunc;
		t_destroyTetrahedronFunc m_destroyTetrahedronFunc;
		
		/** Parameter to be passed to creation function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of PointData.
		*/
		void* m_createParam;

		/** Parameter to be passed to destruction function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of PointData.
		*/
		void* m_destroyParam;
	};

} // namespace topology

} // namespace component

/*
// Specialization of the defaulttype::DataTypeInfo type traits template

namespace defaulttype
{

template<class T, class Alloc>
struct DataTypeInfo< sofa::component::topology::PointData<T,Alloc> > : public VectorTypeInfo<sofa::component::topology::PointData<T,Alloc> >
{
    static std::string name() { std::ostringstream o; o << "PointData<" << DataTypeName<T>::name() << ">"; return o.str(); }
};

} // namespace defaulttype
*/

} // namespace sofa

#endif // _POINTDATA_H_
