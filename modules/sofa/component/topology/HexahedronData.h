/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_H
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONDATA_H

#include <sofa/component/topology/HexahedronSetTopology.h>
#include <sofa/helper/vector.h>
#include <vector>
#include <iostream>


namespace sofa
{

namespace component
{

namespace topology
{

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
        inline void td_basicDestroyFunc(int , void* , T& )
        {
            return;
        }

        /** \brief A class for storing Hexahedron related data. Automatically manages topology changes.
         *
         * This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
         * happen (non exhaustive list: Hexahedra added, removed, fused, renumbered).
         */
        template<
			class T, 
			class Alloc = std::allocator<T>
		> 
		class HexahedronData : public sofa::helper::vector<T, Alloc> {
		public:
			/// size_type
			typedef typename sofa::helper::vector<T,Alloc>::size_type size_type;
			/// reference to a value (read-write)
			typedef typename sofa::helper::vector<T,Alloc>::reference reference;
			/// const reference to a value (read only)
			typedef typename sofa::helper::vector<T,Alloc>::const_reference const_reference;

        private:


            /// Creation function, called when adding elements.
            void (*m_createFunc)(int, void*, T&, const Hexahedron&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >& );

            /// Destruction function, called when deleting elements.
            void (*m_destroyFunc)(int, void*, T&);
			


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
            

        private:
            /// Swaps values at indices i1 and i2.
            void swap( unsigned int i1, unsigned int i2 );



            /// Add some values. Values are added at the end of the vector.
            void add( unsigned int nbHexahedra,
                      const sofa::helper::vector< Hexahedron >& hexahedron,
                      const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors = (sofa::helper::vector< sofa::helper::vector< unsigned int > >)0, 
                      const sofa::helper::vector< sofa::helper::vector< double > >& coefs = (const sofa::helper::vector< sofa::helper::vector< double > >)0 );



            /// Remove the values corresponding to the Hexahedra removed.
            void remove( const sofa::helper::vector<unsigned int> &index );



        public:
			
			/// Constructor
			HexahedronData(size_type n, const T& value): sofa::helper::vector<T,Alloc>(n,value) {}
			/// Constructor
			HexahedronData(int n, const T& value): sofa::helper::vector<T,Alloc>(n,value) {}
			/// Constructor
			HexahedronData(long n, const T& value): sofa::helper::vector<T,Alloc>(n,value) {}
			/// Constructor
			explicit HexahedronData(size_type n): sofa::helper::vector<T,Alloc>(n) {}
			/// Constructor
			HexahedronData(const sofa::helper::vector<T, Alloc>& x): sofa::helper::vector<T,Alloc>(x) {}


#ifdef __STL_MEMBER_TEMPLATES
			/// Constructor
			template <class InputIterator>
			HexahedronData(InputIterator first, InputIterator last): sofa::helper::vector<T,Alloc>(first,last){}
#else /* __STL_MEMBER_TEMPLATES */
			/// Constructor
			HexahedronData(typename HexahedronData<T>::const_iterator first, typename HexahedronData<T,Alloc>::const_iterator last): sofa::helper::vector<T,Alloc>(first,last){}
#endif /* __STL_MEMBER_TEMPLATES */


			/// Optionnaly takes 2 parameters, a creation and a destruction function that will be called when adding/deleting elements.
			HexahedronData( void (*createFunc) (int, void*, T&,const Hexahedron& , const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double >&) = trd_basicCreateFunc,  void* createParam  = (void*)NULL, 
				void (*destroyFunc)(int, void*, T&                                     ) = trd_basicDestroyFunc, void* destroyParam = (void*)NULL ) 
				: sofa::helper::vector<T,Alloc>(), m_createFunc(createFunc), m_destroyFunc(destroyFunc), 
				m_createParam(createParam), m_destroyParam(destroyParam)
			{}


            /// Handle HexahedronSetTopology related events, ignore others.
            void handleTopologyEvents( std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::componentmodel::topology::TopologyChange *>::const_iterator &end );


			void setCreateFunction(void (*createFunc )(int, void*, T&, const Hexahedron&, const sofa::helper::vector< unsigned int >&, const sofa::helper::vector< double>& )) {
				m_createFunc=createFunc;
			}
			void setDestroyFunction( void (*destroyFunc)(int, void*, T&) ) {
				m_destroyFunc=destroyFunc;
			}
			void setDestroyParameter( void* destroyParam ) {
				m_destroyParam=destroyParam;
			}
			void setCreateParameter( void* createParam ) {
				m_createParam=createParam;
			}
            
        };


    } // namespace topology

} // namespace component

} // namespace sofa

#endif // _HexahedronDATA_H_
