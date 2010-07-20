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
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_H
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_H

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/vector.h>

namespace sofa
{

namespace component
{

namespace topology
{
	using core::topology::BaseMeshTopology;
	typedef BaseMeshTopology::Hexa Hexa;
	typedef Hexa Hexahedron;

	/** \brief Basic creation function for element of type T : simply calls default constructor.
	*
	*/
	template < typename T >
	inline void td_basicCreateFunc(int , 
								void* , T& t, 
								const Hexahedron& ,  
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
	inline void hd_basicDestroyFunc(int , void* , T& )
	{
		return;
	}

   /** \brief Topological Engine which will handle all HexahedronData */
   class HexahedronSetTopologyEngine;

	/** \brief A class for storing Hexahedron related data. Automatically manages topology changes.
	*
	* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
	* happen (non exhaustive list: Hexahedra added, removed, fused, renumbered).
	*/
	template< class T, class Alloc = helper::CPUMemoryManager<T> >
	class HexahedronData : public sofa::core::objectmodel::Data<sofa::helper::vector<T, Alloc> >
	{
      friend class HexahedronSetTopologyEngine;

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
      typedef void (*t_createFunc)(int, void*, T&, const Hexahedron&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >& );
      /// Destruction function, called when deleting elements.
      typedef void (*t_destroyFunc)(int, void*, T&);


	public:
      HexahedronData( const typename sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >::InitData& data,
                      void (*createFunc) (int, void*, T&, const Hexahedron &,const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = td_basicCreateFunc,
                      void* createParam  = (void*)NULL,
                      void (*destroyFunc)(int, void*, T&) = hd_basicDestroyFunc,
                      void* destroyParam = (void*)NULL )
                         : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(data),
                         m_createFunc(createFunc), m_destroyFunc(destroyFunc),
                         m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Constructor
		HexahedronData(size_type n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		HexahedronData(int n, const T& value) : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		HexahedronData(long n, const T& value): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n, value);
			this->endEdit();
		}
		/// Constructor
		explicit HexahedronData(size_type n): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->resize(n);
			this->endEdit();
		}
		/// Constructor
		HexahedronData(const sofa::helper::vector<T, Alloc>& x): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			(*data) = x;
			this->endEdit();
		}

#ifdef __STL_MEMBER_TEMPLATES
		/// Constructor
		template <class InputIterator>
            HexahedronData(InputIterator first, InputIterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#else /* __STL_MEMBER_TEMPLATES */
		/// Constructor
		HexahedronData(const_iterator first, const_iterator last): sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false)
		{
			sofa::helper::vector<T, Alloc>* data = this->beginEdit();
			data->assign(first, last);
			this->endEdit();
		}
#endif /* __STL_MEMBER_TEMPLATES */

		/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
		HexahedronData( void (*createFunc) (int, void*, T&,const Hexahedron& , const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = td_basicCreateFunc,  
                      void* createParam  = (void*)NULL,
                      void (*destroyFunc)(int, void*, T&                                     ) = hd_basicDestroyFunc,
                      void* destroyParam = (void*)NULL )
                         : sofa::core::objectmodel::Data< sofa::helper::vector<T, Alloc> >(0, false, false),
                         m_createFunc(createFunc), m_destroyFunc(destroyFunc),
                         m_createParam(createParam), m_destroyParam(destroyParam)
		{}

		/// Handle HexahedronSetTopology related events, ignore others.
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


	private:
		/// Swaps values at indices i1 and i2.
		void swap( unsigned int i1, unsigned int i2 );

		/// Add some values. Values are added at the end of the vector.
		void add( unsigned int nbHexahedra,
                const sofa::helper::vector< Hexahedron >& hexahedron,
                const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors,
                const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

		/// Remove the values corresponding to the Hexahedra removed.
		void remove( const sofa::helper::vector<unsigned int> &index );


	private:
      t_createFunc m_createFunc;
      t_destroyFunc m_destroyFunc;

		/** Parameter to be passed to creation function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of HexahedronData.
		*/
		void* m_createParam;

		/** Parameter to be passed to destruction function.
		*
		* Warning : construction and destruction of this object is not of the responsibility of HexahedronData.
		*/
		void* m_destroyParam;
	};


} // namespace topology

} // namespace component

} // namespace sofa

#endif // _HexahedronDATA_H_
