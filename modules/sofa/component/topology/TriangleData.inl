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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLEDATA_INL

#include <sofa/component/topology/TriangleData.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

namespace sofa
{

namespace component
{

namespace topology
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////implementation//////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template <typename T, typename Alloc>
	void TriangleData<T,Alloc>::handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
                                                     std::list< const core::topology::TopologyChange *>::const_iterator &end )
	{
		while( changeIt != end )
		{
			core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

			switch( changeType ) 
			{
         case core::topology::TETRAHEDRAADDED:
            {
               if (m_createTetrahedronFunc)
               {
                  const TetrahedraAdded *ea=static_cast< const TetrahedraAdded* >( *changeIt );
                  this->applyCreateTetrahedronFunction(ea->tetrahedronIndexArray);
               }
               break;
            }
         case core::topology::TETRAHEDRAREMOVED:
            {
               if (m_destroyTetrahedronFunc)
               {
                  const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
                  this->applyDestroyTetrahedronFunction(er->getArray());
               }
               break;
            }
         case core::topology::TRIANGLESADDED:
            {
               const TrianglesAdded *ta = static_cast< const TrianglesAdded * >( *changeIt );
               add( ta->getNbAddedTriangles(), ta->triangleArray, ta->ancestorsList, ta->coefs );
               break;
            }

			case core::topology::TRIANGLESREMOVED:
            {
               const sofa::helper::vector<unsigned int> &tab = ( static_cast< const TrianglesRemoved *>( *changeIt ) )->getArray();
               remove( tab );
               break;
            }
         case core::topology::QUADSADDED:
            {
               if (m_createQuadFunc)
               {
                  const QuadsAdded *ea=static_cast< const QuadsAdded* >( *changeIt );
                  this->applyCreateQuadFunction(ea->quadIndexArray);
               }
               break;
            }
         case core::topology::QUADSREMOVED:
            {
               if (m_destroyQuadFunc)
               {
                  const QuadsRemoved *er=static_cast< const QuadsRemoved * >( *changeIt );
                  this->applyDestroyQuadFunction(er->getArray());
               }
               break;
            }
         case core::topology::HEXAHEDRAADDED:
            {
               if (m_createHexahedronFunc)
               {
                  const HexahedraAdded *ea=static_cast< const HexahedraAdded* >( *changeIt );
                  this->applyCreateHexahedronFunction(ea->hexahedronIndexArray);
               }
               break;
            }
         case core::topology::HEXAHEDRAREMOVED:
            {
               if (m_destroyHexahedronFunc)
               {
                  const HexahedraRemoved *er=static_cast< const HexahedraRemoved * >( *changeIt );
                  this->applyDestroyHexahedronFunction(er->getArray());
               }
               break;
            }
			case core::topology::TRIANGLESMOVED_REMOVING:
            {
               const sofa::helper::vector< unsigned int >& triList = ( static_cast< const TrianglesMoved_Removing *>( *changeIt ) )->trianglesAroundVertexMoved;
               sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

               for (unsigned int i = 0; i <triList.size(); i++)
                  m_destroyFunc( triList[i], m_destroyParam, data[triList[i]] );

               this->endEdit();
               break;
            }
			case core::topology::TRIANGLESMOVED_ADDING:
            {
               const sofa::helper::vector< unsigned int >& triList = ( static_cast< const TrianglesMoved_Adding *>( *changeIt ) )->trianglesAroundVertexMoved;
               const sofa::helper::vector< Triangle >& triArray = ( static_cast< const TrianglesMoved_Adding *>( *changeIt ) )->triangleArray2Moved;
               sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

               // Recompute data
               sofa::helper::vector< unsigned int > ancestors;
               sofa::helper::vector< double >  coefs;
               coefs.push_back (1.0);
               ancestors.resize(1);

               for (unsigned int i = 0; i <triList.size(); i++)
               {
                  ancestors[0] = triList[i];
                  m_createFunc( triList[i], m_createParam, data[triList[i]], triArray[i], ancestors, coefs );
               }

               this->endEdit();
               break;
            }
			default:
            // Ignore events that are not Triangle or Point related.
            break;
			}; // switch( changeType )

			++changeIt;
		}
	}



   ///////////////////// Public functions to call pointer to fonction ////////////////////////
   /// Apply adding quads elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyCreateQuadFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createQuadFunc)
      {
         (*m_createQuadFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing quads elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyDestroyQuadFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyQuadFunc)
      {
         (*m_destroyQuadFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }

   /// Apply adding tetrahedra elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyCreateTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createTetrahedronFunc)
      {
         (*m_createTetrahedronFunc)(indices,m_createParam, *(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing tetrahedra elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyDestroyTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyTetrahedronFunc)
      {
         (*m_destroyTetrahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }

   /// Apply adding hexahedra elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyCreateHexahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createHexahedronFunc)
      {
         (*m_createHexahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing hexahedra elements.
   template <typename T, typename Alloc>
   void TriangleData<T,Alloc>::applyDestroyHexahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyHexahedronFunc)
      {
         (*m_destroyHexahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }


   ///////////////////// Private functions on TriangleData changes /////////////////////////////

	template <typename T, typename Alloc>
	void TriangleData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;
		this->endEdit();

	}

	template <typename T, typename Alloc>
	void TriangleData<T,Alloc>::add( unsigned int nbTriangles, 
									const sofa::helper::vector< Triangle >& triangle, 
									const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
									const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		// Using default values
		unsigned int i0 = data.size();
		data.resize(i0+nbTriangles);

		for (unsigned int i = 0; i < nbTriangles; ++i)
		{
			T& t = data[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc( i0+i, m_createParam, t, triangle[i], empty_vecint, empty_vecdouble );
			}
			else
				m_createFunc( i0+i, m_createParam, t, triangle[i], ancestors[i], coefs[i] );
		}
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void TriangleData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

		unsigned int last = data.size() -1;

		for (unsigned int i = 0; i < index.size(); ++i)
		{
			m_destroyFunc( index[i], m_destroyParam, data[index[i]] );
			swap( index[i], last );
			--last;
		}

		data.resize( data.size() - index.size() );
	}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // _TriangleDATA_INL_
