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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGEDATA_H
#define SOFA_COMPONENT_TOPOLOGY_EDGEDATA_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/vector.h>
#include <sofa/component/component.h>

namespace sofa
{

namespace component
{

namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Edge Edge;

	/** \brief Basic creation function for element of type T : simply calls default constructor.
	*
	*/
	template < typename T >
	inline void ed_basicCreateFunc(int , 
								void* , T& t, 
								const Edge& ,  
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
	inline void ed_basicDestroyFunc(int , void* , T& )
	{
		return;
	}

   /** \brief Topological Engine which will handle all EdgeData */
   //class EdgeSetTopologyEngine;


	/** \brief A class for storing Edge related data. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: Edges added, removed, fused, renumbered).
	*/
	template< class T, class Alloc = helper::CPUMemoryManager<T> >
	class EdgeData : public sofa::core::objectmodel::Data<sofa::helper::vector<T, Alloc> >
	{
      //friend class EdgeSetTopologyEngine;

	public:
		/// size_type
		typedef typename sofa::helper::vector<T,Alloc>::size_type size_type;
		/// reference to a value (read-write)
		typedef typename sofa::helper::vector<T,Alloc>::reference reference;
		/// const reference to a value (read only)
		typedef typename sofa::helper::vector<T,Alloc>::const_reference const_reference;
		/// const iterator 
		typedef typename sofa::helper::vector<T,Alloc>::const_iterator const_iterator;


      /// Creation function, called when adding elements.
      typedef void (*t_createFunc)(int, void*, T&, const Edge&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >& );
      /// Destruction function, called when deleting elements.
      typedef void (*t_destroyFunc)(int, void*, T&);

      /// Creation function, called when adding triangles elements.
      typedef void (*t_createTriangleFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &);
      /// Destruction function, called when removing triangles elements.
      typedef void (*t_destroyTriangleFunc)(const sofa::helper::vector<unsigned int> &, void*,  helper::vector< T > &);

      /// Creation function, called when adding quads elements.
      typedef void (*t_createQuadFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);
      /// Destruction function, called when removing quads elements.
      typedef void (*t_destroyQuadFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);

      /// Creation function, called when adding tetrahedra elements.
      typedef void (*t_createTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);
      /// Destruction function, called when removing tetrahedra elements.
      typedef void (*t_destroyTetrahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);

      /// Creation function, called when adding hexahedra elements.
      typedef void (*t_createHexahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);
      /// Destruction function, called when removing hexahedra elements.
      typedef void (*t_destroyHexahedronFunc)(const sofa::helper::vector<unsigned int> &, void*, helper::vector< T > &);


	public:
		/// Constructor
      EdgeData( const typename sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >::InitData& data,
                void (*createFunc) (int, void*, T&, const Edge &,const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = ed_basicCreateFunc,
                void* createParam  = (void*)NULL,
                void (*destroyFunc)(int, void*, T&) = ed_basicDestroyFunc,
                void* destroyParam = (void*)NULL )
                   : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(data),
                   m_createFunc(createFunc), m_destroyFunc(destroyFunc),
                   m_createTriangleFunc(0), m_destroyTriangleFunc(0),
                   m_createQuadFunc(0), m_destroyQuadFunc(0),
                   m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
                   m_createHexahedronFunc(0), m_destroyHexahedronFunc(0),
                   m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Constructor
		EdgeData(size_type n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		EdgeData(int n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		EdgeData(long n, const T& value): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		explicit EdgeData(size_type n): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n);
			this->endEdit();
		}
		/// Constructor
		EdgeData(const sofa::helper::vector<T, Alloc>& x): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			(*data) = x;
			this->endEdit();
		}

#ifdef __STL_MEMBER_TEMPLATES
		/// Constructor
		template <class InputIterator>
            EdgeData(InputIterator first, InputIterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#else /* __STL_MEMBER_TEMPLATES */
		/// Constructor
		EdgeData(const_iterator first, const_iterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#endif /* __STL_MEMBER_TEMPLATES */


		/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		EdgeData( void (*createFunc) (int, void*, T&, const Edge &,const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = ed_basicCreateFunc,
                void* createParam  = (void*)NULL,
                void (*destroyFunc)(int, void*, T&) = ed_basicDestroyFunc,
                void* destroyParam = (void*)NULL )
                   : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false),
                   m_createFunc(createFunc), m_destroyFunc(destroyFunc),
                   m_createTriangleFunc(0), m_destroyTriangleFunc(0),
                   m_createQuadFunc(0), m_destroyQuadFunc(0),
                   m_createTetrahedronFunc(0), m_destroyTetrahedronFunc(0),
                   m_createHexahedronFunc(0), m_destroyHexahedronFunc(0),
                   m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Handle EdgeSetTopology related events, ignore others.
		void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
                                 std::list< const core::topology::TopologyChange *>::const_iterator &end );


      /// Creation function, called when adding elements.
      void setCreateFunction(t_createFunc createFunc)
      {
         m_createFunc=createFunc;
      }
      /// Destruction function, called when deleting elements.
      void setDestroyFunction(t_destroyFunc destroyFunc)
      {
         m_destroyFunc=destroyFunc;
      }

      /// Creation function, called when adding triangles elements.
      void setCreateTriangleFunction(t_createTriangleFunc createTriangleFunc)
      {
         m_createTriangleFunc=createTriangleFunc;
      }
      /// Destruction function, called when removing triangles elements.
      void setDestroyTriangleFunction(t_destroyTriangleFunc destroyTriangleFunc)
      {
         m_destroyTriangleFunc=destroyTriangleFunc;
      }

      /// Creation function, called when adding quads elements.
      void setCreateQuadFunction(t_createQuadFunc createQuadFunc)
      {
         m_createQuadFunc=createQuadFunc;
      }
      /// Destruction function, called when removing quads elements.
      void setDestroyQuadFunction(t_destroyQuadFunc destroyQuadFunc)
      {
         m_destroyQuadFunc=destroyQuadFunc;
      }

      /// Creation function, called when adding tetrahedra elements.
      void setCreateTetrahedronFunction(t_createTetrahedronFunc createTetrahedronFunc)
      {
         m_createTetrahedronFunc=createTetrahedronFunc;
      }
      /// Destruction function, called when removing tetrahedra elements.
      void setDestroyTetrahedronFunction(t_destroyTetrahedronFunc destroyTetrahedronFunc)
      {
         m_destroyTetrahedronFunc=destroyTetrahedronFunc;
      }

      /// Creation function, called when adding hexahedra elements.
      void setCreateHexahedronFunction(t_createHexahedronFunc createHexahedronFunc)
      {
         m_createHexahedronFunc=createHexahedronFunc;
      }
      /// Destruction function, called when removing hexahedra elements.
      void setDestroyHexahedronFunction(t_destroyHexahedronFunc destroyHexahedronFunc)
      {
         m_destroyHexahedronFunc=destroyHexahedronFunc;
      }

      /// Creation function, called when adding parameter to those elements.
      void setDestroyParameter( void* destroyParam )
      {
         m_destroyParam=destroyParam;
      }
      /// Destruction function, called when removing parameter to those elements.
      void setCreateParameter( void* createParam )
      {
         m_createParam=createParam;
      }


      /** Public fonction to apply creation and destruction functions */
      /// Apply adding triangles elements.
      void applyCreateTriangleFunction(const sofa::helper::vector<unsigned int> & indices);
      /// Apply removing triangles elements.
      void applyDestroyTriangleFunction(const sofa::helper::vector<unsigned int> & indices);

      /// Apply adding quads elements.
      void applyCreateQuadFunction(const sofa::helper::vector<unsigned int> & indices);
      /// Apply removing quads elements.
      void applyDestroyQuadFunction(const sofa::helper::vector<unsigned int> & indices);

      /// Apply adding tetrahedra elements.
      void applyCreateTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices);
      /// Apply removing tetrahedra elements.
      void applyDestroyTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices);

      /// Apply adding hexahedra elements.
      void applyCreateHexahedronFunction(const sofa::helper::vector<unsigned int> & indices);
      /// Apply removing hexahedra elements.
      void applyDestroyHexahedronFunction(const sofa::helper::vector<unsigned int> & indices);


		T& operator[](int i)
      {
			sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
			T& result = data[i];
			this->endEdit();
         return result;
      }

   //private:
		/// Swaps values at indices i1 and i2.
		void swap( unsigned int i1, unsigned int i2 );

		/// Add some values. Values are added at the end of the vector.
		void add( unsigned int nbEdges,
                const sofa::helper::vector< Edge >& edge,
                const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors,
                const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

		/// Remove the values corresponding to the Edges removed.
		void remove( const sofa::helper::vector<unsigned int> &index );


   //private:
      t_createFunc m_createFunc;
      t_destroyFunc m_destroyFunc;
      t_createTriangleFunc m_createTriangleFunc;
      t_destroyTriangleFunc m_destroyTriangleFunc;
      t_createQuadFunc m_createQuadFunc;
      t_destroyQuadFunc m_destroyQuadFunc;
      t_createTetrahedronFunc m_createTetrahedronFunc;
      t_destroyTetrahedronFunc m_destroyTetrahedronFunc;
      t_createHexahedronFunc m_createHexahedronFunc;
      t_destroyHexahedronFunc m_destroyHexahedronFunc;

   private:
		/** Parameter to be passed to creation function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of EdgeData.
		*/
		void* m_createParam;

		/** Parameter to be passed to destruction function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of EdgeData.
		*/
		void* m_destroyParam;
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_EDGEDATA_H
