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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_INL
#define SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_INL
#include <sofa/component/topology/PointSetTopologyEngine.h>

#include <sofa/component/topology/PointSetTopologyChange.h>
#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/HexahedronSetTopologyContainer.h>

#include <sofa/component/topology/PointData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   template <typename T, typename VecT>
   PointSetTopologyEngine<T,VecT>::PointSetTopologyEngine() : sofa::core::topology::TopologyEngine(),
   m_topology(NULL),
   m_pointsLinked(false), m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }


   template <typename T, typename VecT>
   PointSetTopologyEngine<T,VecT>::PointSetTopologyEngine(PointData<T, VecT> *_topologicalData) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_pointsLinked(false), m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }

   template <typename T, typename VecT>
   PointSetTopologyEngine<T,VecT>::PointSetTopologyEngine(PointData<T, VecT> *_topologicalData, sofa::core::topology::BaseMeshTopology *_topology) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_pointsLinked(false), m_edgesLinked(false), m_trianglesLinked(false),
   m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;

      this->init();
   }


   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::init()
   {
      this->addInput(&m_changeList);
      this->addOutput(m_topologicalData);
      //sofa::core::topology::TopologyEngine::init();

      if (m_topology)
         m_topology->addTopologyEngine(this);
   }


   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::reinit()
   {
      this->update();
   }

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::update()
   {
      std::cout << "PointSetTopologyEngine::update()" << std::endl;
      std::cout<< "Number of topological changes: " << m_changeList.getValue().size() << std::endl;
      this->ApplyTopologyChanges();
      std::cout << "PointSetTopologyEngine::update() - end" << std::endl;
   }


   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::registerTopology(sofa::core::topology::BaseMeshTopology *_topology)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;
   }



   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::ApplyTopologyChanges()
   {
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      std::cout<< "Number of topological changes: " << m_changeList.getValue().size() << std::endl;

      for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
      {
         core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

         switch( changeType ) // See enum events in sofa::core::topology::Topology
         {
            // Events concerning Points
         case core::topology::POINTSREMOVED:
            {
               const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRemoved * >( *changeIt ) )->getArray();
               m_topologicalData->remove( tab );
               break;
            }
         case core::topology::POINTSADDED:
            {
               unsigned int nbPoints = ( static_cast< const PointsAdded * >( *changeIt ) )->getNbAddedVertices();
               sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( static_cast< const PointsAdded * >( *changeIt ) )->ancestorsList;
               sofa::helper::vector< sofa::helper::vector< double       > > coefs     = ( static_cast< const PointsAdded * >( *changeIt ) )->coefs;
               m_topologicalData->add( nbPoints, ancestors, coefs );
               break;
            }
         case core::topology::POINTSINDICESSWAP:
            {
               unsigned int i1 = ( static_cast< const PointsIndicesSwap * >( *changeIt ) )->index[0];
               unsigned int i2 = ( static_cast< const PointsIndicesSwap* >( *changeIt ) )->index[1];
               m_topologicalData->swap( i1, i2 );
               break;
            }
         case core::topology::POINTSMOVED:
            {
               const sofa::helper::vector< unsigned int >& indexList = ( static_cast< const PointsMoved * >( *changeIt ) )->indicesList;
               const sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors = ( static_cast< const PointsMoved * >( *changeIt ) )->ancestorsList;
               sofa::helper::vector< sofa::helper::vector< double > > coefs = ( static_cast< const PointsMoved * >( *changeIt ) )->baryCoefsList;

               m_topologicalData->move( indexList, ancestors, coefs);
               break;
            }
         case core::topology::POINTSRENUMBERING:
            {
               const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRenumbering * >( *changeIt ) )->getIndexArray();
               m_topologicalData->renumber( tab );
               break;
            }
            // Events concerning Edges
         case core::topology::EDGESADDED:
            {
               if (m_topologicalData->m_createEdgeFunc)
               {
                  const EdgesAdded *ea=static_cast< const EdgesAdded* >( *changeIt );
                  m_topologicalData->applyCreateEdgeFunction(ea->edgeIndexArray);
               }
               break;
            }
         case core::topology::EDGESREMOVED:
            {
               if (m_topologicalData->m_destroyEdgeFunc)
               {
                  const EdgesRemoved *er=static_cast< const EdgesRemoved * >( *changeIt );
                  m_topologicalData->applyDestroyEdgeFunction(er->getArray());
               }
               break;
            }
         case core::topology::EDGESMOVED_REMOVING:
            {
               if (m_topologicalData->m_destroyEdgeFunc)
               {
                  const EdgesMoved_Removing *em=static_cast< const EdgesMoved_Removing * >( *changeIt );
                  m_topologicalData->applyDestroyEdgeFunction(em->edgesAroundVertexMoved);
               }
               break;
            }
         case core::topology::EDGESMOVED_ADDING:
            {
               if (m_topologicalData->m_createEdgeFunc)
               {
                  const EdgesMoved_Adding *em=static_cast< const EdgesMoved_Adding* >( *changeIt );
                  m_topologicalData->applyCreateEdgeFunction(em->edgesAroundVertexMoved);
               }
               break;
            }
         case core::topology::EDGESRENUMBERING:
            {
#ifndef NDEBUG
               sout << "EDGESRENUMBERING topological change is not implemented in PointSetTopologyEngine" << sendl;
#endif
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
               sout << "TRIANGLESRENUMBERING topological change is not implemented in PointSetTopologyEngine" << sendl;
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
               sout << "QUADSRENUMBERING topological change is not implemented in PointSetTopologyEngine" << sendl;
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
               sout << "TETRAHEDRARENUMBERING topological change is not implemented in PointSetTopologyEngine" << sendl;
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
               sout << "HEXAHEDRARENUMBERING topological change is not implemented in PointSetTopologyEngine" << sendl;
#endif
               break;
            }
         default:
   break;
   }

   }

      m_changeList.endEdit();
   }

/*
   void PointSetTopologyEngine::addTopologicalData(PointData<void *> &topologicalData)
   {
      m_topologicalData.push_back(&topologicalData);
   }

   void PointSetTopologyEngine::removeTopoligicalData(PointData<void *> &topologicalData)
   {
      m_topologicalData.remove(&topologicalData);
   }
*/






   /// Creation function, called when adding elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateFunction(t_createFunc createFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateFunction(createFunc);

         if (!m_pointsLinked && m_topology) // No link
            this->linkToPointDataArray();
      }
   }

   /// Destruction function, called when deleting elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyFunction(t_destroyFunc destroyFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyFunction(destroyFunc);

         if (!m_pointsLinked && m_topology) // No link
            this->linkToPointDataArray();
      }
   }

   /// Creation function, called when adding edges elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateEdgeFunction(t_createEdgeFunc createEdgeFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setCreateEdgeFunction(createEdgeFunc);

         if (!m_edgesLinked && m_topology) // No link
            this->linkToEdgeDataArray();
      }
   }

   /// Destruction function, called when removing edges elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyEdgeFunction(t_destroyEdgeFunc destroyEdgeFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To PointData
         m_topologicalData->setDestroyEdgeFunction(destroyEdgeFunc);

         if (!m_edgesLinked && m_topology) // No link
            this->linkToEdgeDataArray();
      }
   }

   /// Creation function, called when adding triangles elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateTriangleFunction(t_createTriangleFunc createTriangleFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyTriangleFunction(t_destroyTriangleFunc destroyTriangleFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateQuadFunction(t_createQuadFunc createQuadFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyQuadFunction(t_destroyQuadFunc destroyQuadFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateTetrahedronFunction(t_createTetrahedronFunc createTetrahedronFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyTetrahedronFunction(t_destroyTetrahedronFunc destroyTetrahedronFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateHexahedronFunction(t_createHexahedronFunc createHexahedronFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyHexahedronFunction(t_destroyHexahedronFunc destroyHexahedronFunc)
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
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setDestroyParameter( void* destroyParam )
   {
      if (m_topologicalData)
         m_topologicalData->setDestroyParameter(destroyParam);
   }

   /// Destruction function, called when removing parameter to those elements.
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::setCreateParameter( void* createParam )
   {
      if (m_topologicalData)
         m_topologicalData->setCreateParameter(createParam);
   }



   /// Function to link DataEngine with Data array from topology
   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToPointDataArray()
   {
      sofa::component::topology::PointSetTopologyContainer* _container = dynamic_cast<sofa::component::topology::PointSetTopologyContainer*>(m_topology);

      if (_container == NULL)
      {
         std::cerr <<"Error: Can't dynamic cast topology as PointSetTopologyContainer" << std::endl;
         return;
      }

      (_container->getPointDataArray()).addOutput(this);
      m_pointsLinked = true;
   }

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToEdgeDataArray()
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

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToTriangleDataArray()
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

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToQuadDataArray()
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

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToTetrahedronDataArray()
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

   template <typename T, typename VecT>
   void PointSetTopologyEngine<T,VecT>::linkToHexahedronDataArray()
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



#endif // SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_INL
