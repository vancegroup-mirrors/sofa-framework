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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGEDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_EDGEDATA_INL

#include <sofa/component/topology/EdgeData.h>
#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>	
#include <sofa/component/topology/TetrahedronSetTopologyChange.h> 
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

namespace sofa
{

namespace component
{

namespace topology
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////implementation//////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template <typename T, typename Alloc>
	void EdgeData<T,Alloc>::handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, 
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
               if (m_createTriangleFunc)
               {
                  const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
                  this->applyCreateTriangleFunction(ea->triangleIndexArray);
               }
               break;
            }
         case core::topology::TRIANGLESREMOVED:
            {
               if (m_destroyTriangleFunc)
               {
                  const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
                  this->applyDestroyTriangleFunction(er->getArray());
               }
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
         case core::topology::EDGESADDED:
            {
               const EdgesAdded *ea=static_cast< const EdgesAdded * >( *changeIt );
               add( ea->getNbAddedEdges(), ea->edgeArray, ea->ancestorsList, ea->coefs );
               break;
            }
         case core::topology::EDGESREMOVED:
            {
               const sofa::helper::vector<unsigned int> &tab = ( static_cast< const EdgesRemoved *>( *changeIt ) )->getArray();
               remove( tab );
               break;
            }
         case core::topology::EDGESMOVED_REMOVING:
            {
               const sofa::helper::vector< unsigned int >& edgeList = ( static_cast< const EdgesMoved_Removing *>( *changeIt ) )->edgesAroundVertexMoved;
               sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

               for (unsigned int i = 0; i <edgeList.size(); i++)
                  m_destroyFunc( edgeList[i], m_destroyParam, data[edgeList[i]] );

               this->endEdit();
               break;
            }
         case core::topology::EDGESMOVED_ADDING:
            {
               const sofa::helper::vector< unsigned int >& edgeList = ( static_cast< const EdgesMoved_Adding *>( *changeIt ) )->edgesAroundVertexMoved;
               const sofa::helper::vector< Edge >& edgeArray = ( static_cast< const EdgesMoved_Adding *>( *changeIt ) )->edgeArray2Moved;
               sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());

               // Recompute data
               sofa::helper::vector< unsigned int > ancestors;
               sofa::helper::vector< double >  coefs;
               coefs.push_back (1.0);
               ancestors.resize(1);

               for (unsigned int i = 0; i <edgeList.size(); i++)
               {
                  ancestors[0] = edgeList[i];
                  m_createFunc( edgeList[i], m_createParam, data[edgeList[i]], edgeArray[i], ancestors, coefs );
               }

               this->endEdit();
               break;
            }
         case core::topology::TRIANGLESMOVED_REMOVING:
            {
               if (m_destroyTriangleFunc)
               {
                  const TrianglesMoved_Removing *tm=static_cast< const TrianglesMoved_Removing* >( *changeIt );
                  (*m_destroyTriangleFunc)(tm->trianglesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
                  this->endEdit();
               }

               break;
            }
         case core::topology::TRIANGLESMOVED_ADDING:
            {
               if (m_createTriangleFunc)
               {
                  const TrianglesMoved_Adding *tm=static_cast< const TrianglesMoved_Adding * >( *changeIt );
                  (*m_createTriangleFunc)(tm->trianglesAroundVertexMoved,m_createParam,*(this->beginEdit() ) );
                  this->endEdit();
               }

               break;
            }
         default:
            // Ignore events that are not Edge or Point related.
            break;
			}; // switch( changeType )

			++changeIt;
		} 
	}



   ///////////////////// Public functions to call pointer to fonction ////////////////////////
   /// Apply adding triangles elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyCreateTriangleFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createTriangleFunc)
      {
         (*m_createTriangleFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing triangles elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyDestroyTriangleFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyTriangleFunc)
      {
         (*m_destroyTriangleFunc)(indices,m_createParam, *(this->beginEdit() ) );
         this->endEdit();
      }
   }

   /// Apply adding quads elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyCreateQuadFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createQuadFunc)
      {
         (*m_createQuadFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing quads elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyDestroyQuadFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyQuadFunc)
      {
         (*m_destroyQuadFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }

   /// Apply adding tetrahedra elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyCreateTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createTetrahedronFunc)
      {
         (*m_createTetrahedronFunc)(indices,m_createParam, *(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing tetrahedra elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyDestroyTetrahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyTetrahedronFunc)
      {
         (*m_destroyTetrahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }

   /// Apply adding hexahedra elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyCreateHexahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_createHexahedronFunc)
      {
         (*m_createHexahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }
   /// Apply removing hexahedra elements.
   template <typename T, typename Alloc>
   void EdgeData<T,Alloc>::applyDestroyHexahedronFunction(const sofa::helper::vector<unsigned int> & indices)
   {
      if (m_destroyHexahedronFunc)
      {
         (*m_destroyHexahedronFunc)(indices,m_createParam,*(this->beginEdit() ) );
         this->endEdit();
      }
   }


   ///////////////////// Private functions on EdgeData changes /////////////////////////////


	template <typename T, typename Alloc>
	void EdgeData<T,Alloc>::swap( unsigned int i1, unsigned int i2 ) 
	{
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		T tmp = data[i1];
		data[i1] = data[i2];
		data[i2] = tmp;
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void EdgeData<T,Alloc>::add( unsigned int nbEdges, 
								const sofa::helper::vector< Edge >& edge, 
								const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors, 
								const sofa::helper::vector< sofa::helper::vector< double > >& coefs ) 
	{
		// Using default values
		sofa::helper::vector<T, Alloc>& data = *(this->beginEdit());
		unsigned int i0 = data.size();
		data.resize(i0+nbEdges);

		for (unsigned int i = 0; i < nbEdges; ++i)
		{
			T& t = data[i0+i];
			if (ancestors.empty() || coefs.empty())
			{
				const sofa::helper::vector< unsigned int > empty_vecint;
				const sofa::helper::vector< double > empty_vecdouble;
				m_createFunc( i0+i, m_createParam, t, edge[i], empty_vecint, empty_vecdouble);
			}
			else
				m_createFunc( i0+i, m_createParam, t, edge[i], ancestors[i], coefs[i] );
		}
		this->endEdit();
	}

	template <typename T, typename Alloc>
	void EdgeData<T,Alloc>::remove( const sofa::helper::vector<unsigned int> &index ) 
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
		this->endEdit();
		//sout << "EdgeData: vector has now "<<this->size()<<" entries."<<sendl; 
	}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_EDGEDATA_INL
