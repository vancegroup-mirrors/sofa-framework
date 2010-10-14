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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_INL
#define SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_INL
#include <sofa/component/topology/EdgeSetTopologyEngine.h>

#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/HexahedronSetTopologyContainer.h>

#include <sofa/component/topology/EdgeData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   template <typename T, typename Alloc>
   EdgeSetTopologyEngine<T,Alloc>::EdgeSetTopologyEngine() : sofa::core::topology::TopologyEngine(),
   m_topology(NULL),
   m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }


   template <typename T, typename Alloc>
   EdgeSetTopologyEngine<T,Alloc>::EdgeSetTopologyEngine(EdgeData<T, Alloc> *_topologicalData) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }

   template <typename T, typename Alloc>
   EdgeSetTopologyEngine<T,Alloc>::EdgeSetTopologyEngine(EdgeData<T, Alloc> *_topologicalData, sofa::core::topology::BaseMeshTopology *_topology) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;

      this->init();
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::init()
   {
      this->addInput(&m_changeList);
      this->addOutput(m_topologicalData);
      //sofa::core::topology::TopologyEngine::init();

      if (m_topology)
         m_topology->addTopologyEngine(this);
   }


   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::reinit()
   {
      this->update();
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::update()
   {
      std::cout << "EdgeSetTopologyEngine::update()" << std::endl;
      std::cout<< "Number of topological changes: " << m_changeList.getValue().size() << std::endl;
      this->ApplyTopologyChanges();
      std::cout << "EdgeSetTopologyEngine::update() - end" << std::endl;
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::registerTopology(sofa::core::topology::BaseMeshTopology *_topology)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;
   }



   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::ApplyTopologyChanges()
   {
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
      {
         core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

         switch( changeType ) // See enum events in sofa::core::topology::Topology
         {
            // Events concerning Edges
         case core::topology::EDGESADDED:
            {
               const EdgesAdded *ea=static_cast< const EdgesAdded * >( *changeIt );
               m_topologicalData->add( ea->getNbAddedEdges(), ea->edgeArray, ea->ancestorsList, ea->coefs );
               break;
            }
         case core::topology::EDGESREMOVED:
            {
               const sofa::helper::vector<unsigned int> &tab = ( static_cast< const EdgesRemoved *>( *changeIt ) )->getArray();
               m_topologicalData->remove( tab );
               break;
            }
         case core::topology::EDGESMOVED_REMOVING:
            {
               const sofa::helper::vector< unsigned int >& edgeList = ( static_cast< const EdgesMoved_Removing *>( *changeIt ) )->edgesAroundVertexMoved;
               m_topologicalData->remove( edgeList );
               break;
            }
         case core::topology::EDGESMOVED_ADDING:
            {
               const sofa::helper::vector< unsigned int >& edgeList = ( static_cast< const EdgesMoved_Adding *>( *changeIt ) )->edgesAroundVertexMoved;
               const sofa::helper::vector< Edge >& edgeArray = ( static_cast< const EdgesMoved_Adding *>( *changeIt ) )->edgeArray2Moved;

               // Recompute data
               sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors;
               sofa::helper::vector< sofa::helper::vector< double > > coefs;
               coefs.resize(edgeList.size());
               ancestors.resize(edgeList.size());

               for (unsigned int i = 0; i <edgeList.size(); i++)
               {
                  ancestors[i].push_back(1);
                  ancestors[i].push_back(edgeList[i]);
               }

               m_topologicalData->add( edgeList.size(), edgeArray, ancestors, coefs);
               break;
            }
         case core::topology::EDGESRENUMBERING:
            {
               /// FUNCTION MISSING HERE ///

               /*const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRenumbering * >( *changeIt ) )->getIndexArray();
                  renumber( tab );                  */
               break;
            }
            // Events concerning Triangles
         case core::topology::TRIANGLESADDED:
            {
               if (m_topologicalData->m_createTriangleFunc)
               {
                  const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
                  m_topologicalData->applyCreateTriangleFunction(ea->triangleIndexArray);
               }
               break;
            }
         case core::topology::TRIANGLESREMOVED:
            {
               if (m_topologicalData->m_destroyTriangleFunc)
               {
                  const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
                  m_topologicalData->applyDestroyTriangleFunction(er->getArray());
               }
               break;
            }
         case core::topology::TRIANGLESMOVED_REMOVING:
            {
               if (m_topologicalData->m_destroyTriangleFunc)
               {
                  const TrianglesMoved_Removing *tm=static_cast< const TrianglesMoved_Removing* >( *changeIt );
                  m_topologicalData->applyDestroyTriangleFunction(tm->trianglesAroundVertexMoved);
               }
               break;
            }
         case core::topology::TRIANGLESMOVED_ADDING:
            {
               if (m_topologicalData->m_createTriangleFunc)
               {
                  const TrianglesMoved_Adding *tm=static_cast< const TrianglesMoved_Adding * >( *changeIt );
                  m_topologicalData->applyCreateTriangleFunction(tm->trianglesAroundVertexMoved);
               }
               break;
            }
         case core::topology::TRIANGLESRENUMBERING:
            {
#ifndef NDEBUG
               sout << "TRIANGLESRENUMBERING topological change is not implemented in EdgeSetTopologyEngine" << sendl;
#endif
               break;
            }
            // Events concerning Quads
         case core::topology::QUADSADDED:
            {
               if (m_topologicalData->m_createQuadFunc)
               {
                  const QuadsAdded *ea=static_cast< const QuadsAdded* >( *changeIt );
                  m_topologicalData->applyCreateQuadFunction(ea->quadIndexArray);
               }
               break;
            }
         case core::topology::QUADSREMOVED:
            {
               if (m_topologicalData->m_destroyQuadFunc)
               {
                  const QuadsRemoved *er=static_cast< const QuadsRemoved * >( *changeIt );
                  m_topologicalData->applyDestroyQuadFunction(er->getArray());
               }
               break;
            }
         case core::topology::QUADSRENUMBERING:
            {
#ifndef NDEBUG
               sout << "QUADSRENUMBERING topological change is not implemented in EdgeSetTopologyEngine" << sendl;
#endif
               break;
            }
            // Events concerning Tetrahedra
         case core::topology::TETRAHEDRAADDED:
            {
               if (m_topologicalData->m_createTetrahedronFunc)
               {
                  const TetrahedraAdded *ea=static_cast< const TetrahedraAdded* >( *changeIt );
                  m_topologicalData->applyCreateTetrahedronFunction(ea->tetrahedronIndexArray);
               }
               break;
            }
         case core::topology::TETRAHEDRAREMOVED:
            {
               if (m_topologicalData->m_destroyTetrahedronFunc)
               {
                  const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
                  m_topologicalData->applyDestroyTetrahedronFunction(er->getArray());
               }
               break;
            }
         case core::topology::TETRAHEDRARENUMBERING:
            {
#ifndef NDEBUG
               sout << "TETRAHEDRARENUMBERING topological change is not implemented in EdgeSetTopologyEngine" << sendl;
#endif
               break;
            }
            // Events concerning Hexahedra
         case core::topology::HEXAHEDRAADDED:
            {
               if (m_topologicalData->m_createHexahedronFunc)
               {
                  const HexahedraAdded *ea=static_cast< const HexahedraAdded* >( *changeIt );
                  m_topologicalData->applyCreateHexahedronFunction(ea->hexahedronIndexArray);
               }
               break;
            }
         case core::topology::HEXAHEDRAREMOVED:
            {
               if (m_topologicalData->m_destroyHexahedronFunc)
               {
                  const HexahedraRemoved *er=static_cast< const HexahedraRemoved * >( *changeIt );
                  m_topologicalData->applyDestroyHexahedronFunction(er->getArray());
               }
               break;
            }
         case core::topology::HEXAHEDRARENUMBERING:
            {
#ifndef NDEBUG
               sout << "HEXAHEDRARENUMBERING topological change is not implemented in EdgeSetTopologyEngine" << sendl;
#endif
               break;
            }
         default:
            break;
         }
      }

      m_changeList.endEdit();
   }



   /// Creation function, called when adding elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateFunction(t_createFunc createFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To EdgeData
         m_topologicalData->setCreateFunction(createFunc);

         if (!m_edgesLinked && m_topology) // No link
            this->linkToEdgeDataArray();
      }
   }

   /// Destruction function, called when deleting elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyFunction(t_destroyFunc destroyFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To EdgeData
         m_topologicalData->setDestroyFunction(destroyFunc);

         if (!m_edgesLinked && m_topology) // No link
            this->linkToEdgeDataArray();
      }
   }

   /// Creation function, called when adding triangles elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateTriangleFunction(t_createTriangleFunc createTriangleFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateTriangleFunction(createTriangleFunc);

         if (!m_trianglesLinked && m_topology) // No link
            this->linkToTriangleDataArray();
      }
   }

   /// Destruction function, called when removing triangles elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyTriangleFunction(t_destroyTriangleFunc destroyTriangleFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyTriangleFunction(destroyTriangleFunc);

         if (!m_trianglesLinked && m_topology) // No link
            this->linkToTriangleDataArray();
      }
   }

   /// Creation function, called when adding quads elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateQuadFunction(t_createQuadFunc createQuadFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateQuadFunction(createQuadFunc);

         if (!m_quadsLinked && m_topology) // No link
            this->linkToQuadDataArray();
      }
   }

   /// Destruction function, called when removing quads elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyQuadFunction(t_destroyQuadFunc destroyQuadFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyQuadFunction(destroyQuadFunc);

         if (!m_quadsLinked && m_topology) // No link
            this->linkToQuadDataArray();
      }
   }

   /// Creation function, called when adding tetrahedra elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateTetrahedronFunction(t_createTetrahedronFunc createTetrahedronFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateTetrahedronFunction(createTetrahedronFunc);

         if (!m_tetrahedraLinked && m_topology) // No link
            this->linkToTetrahedronDataArray();
      }
   }

   /// Destruction function, called when removing tetrahedra elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyTetrahedronFunction(t_destroyTetrahedronFunc destroyTetrahedronFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyTetrahedronFunction(destroyTetrahedronFunc);

         if (!m_tetrahedraLinked && m_topology) // No link
            this->linkToTetrahedronDataArray();
      }
   }

   /// Creation function, called when adding hexahedra elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateHexahedronFunction(t_createHexahedronFunc createHexahedronFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateHexahedronFunction(createHexahedronFunc);

         if (!m_hexahedraLinked && m_topology) // No link
            this->linkToHexahedronDataArray();
      }
   }

   /// Destruction function, called when removing hexahedra elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyHexahedronFunction(t_destroyHexahedronFunc destroyHexahedronFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyHexahedronFunction(destroyHexahedronFunc);

         if (!m_hexahedraLinked && m_topology) // No link
            this->linkToHexahedronDataArray();
      }
   }

   /// Creation function, called when adding parameter to those elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setDestroyParameter( void* destroyParam )
   {
      if (m_topologicalData)
         m_topologicalData->setDestroyParameter(destroyParam);
   }

   /// Destruction function, called when removing parameter to those elements.
   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::setCreateParameter( void* createParam )
   {
      if (m_topologicalData)
         m_topologicalData->setCreateParameter(createParam);
   }


   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::linkToEdgeDataArray()
   {
      sofa::component::topology::EdgeSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::EdgeSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as EdgeSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getEdgeDataArray()).addOutput(this);
      m_edgesLinked = true;
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::linkToTriangleDataArray()
   {
      sofa::component::topology::TriangleSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as TriangleSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getTriangleDataArray()).addOutput(this);
      m_trianglesLinked = true;
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::linkToQuadDataArray()
   {
      sofa::component::topology::QuadSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::QuadSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as QuadSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getQuadDataArray()).addOutput(this);
      m_quadsLinked = true;
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::linkToTetrahedronDataArray()
   {
      sofa::component::topology::TetrahedronSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::TetrahedronSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as TetrahedronSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getTetrahedronDataArray()).addOutput(this);
      m_tetrahedraLinked = true;
   }

   template <typename T, typename Alloc>
   void EdgeSetTopologyEngine<T,Alloc>::linkToHexahedronDataArray()
   {
      sofa::component::topology::HexahedronSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::HexahedronSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as HexahedronSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getHexahedronDataArray()).addOutput(this);
      m_hexahedraLinked = true;
   }

}// namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_CPP
