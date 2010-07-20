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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_CPP
#define SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_CPP
#include <sofa/component/topology/EdgeSetTopologyEngine.h>
#include <sofa/component/topology/EdgeSetTopologyContainer.h> // should not be include here!!

#include <sofa/component/topology/EdgeSetTopologyChange.h>
#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/EdgeData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   EdgeSetTopologyEngine::EdgeSetTopologyEngine() : sofa::core::topology::TopologyEngine()
   {

   }

   void EdgeSetTopologyEngine::init()
   {
      std::cout << "EdgeSetTopologyEngine::init()" << std::endl;
      sofa::component::topology::PointSetTopologyContainer* topo_container;
      this->getContext()->get(topo_container);

      if (!topo_container)
      {
         std::cerr << "Error in EdgeSetTopologyEngine: PointSetTopologyContainer not found." << std::endl;
         return;
      }

      this->m_changeList = topo_container->getChangeList();

      addInput(&m_changeList);

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
         addOutput((*it));

      std::cout << "EdgeSetTopologyEngine::init() - end" << std::endl;
   }


   void EdgeSetTopologyEngine::reinit()
   {
      this->update();
   }

   void EdgeSetTopologyEngine::update()
   {
      std::cout << "EdgeSetTopologyEngine::update()" << std::endl;
      cleanDirty();
      std::cout << "EdgeSetTopologyEngine::update() - end" << std::endl;
   }


   void EdgeSetTopologyEngine::handleTopologyChange()
   {
      std::cout << "EdgeSetTopologyEngine::update()" << std::endl;
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
      {
         for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
         {
            core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

            switch( changeType ) // See enum events in sofa::core::topology::Topology
            {
               // Events concerning Edges
            case core::topology::EDGESADDED:
               {
                  const EdgesAdded *ea=static_cast< const EdgesAdded * >( *changeIt );
                  (*it)->add( ea->getNbAddedEdges(), ea->edgeArray, ea->ancestorsList, ea->coefs );
                  break;
               }
            case core::topology::EDGESREMOVED:
               {
                  const sofa::helper::vector<unsigned int> &tab = ( static_cast< const EdgesRemoved *>( *changeIt ) )->getArray();
                  (*it)->remove( tab );
                  break;
               }
            case core::topology::EDGESMOVED_REMOVING:
               {
                  const sofa::helper::vector< unsigned int >& edgeList = ( static_cast< const EdgesMoved_Removing *>( *changeIt ) )->edgesAroundVertexMoved;
                  (*it)->remove( edgeList );
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

                  (*it)->add( edgeList.size(), edgeArray, ancestors, coefs);
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
                  if ((*it)->m_createTriangleFunc)
                  {
                     const TrianglesAdded *ea=static_cast< const TrianglesAdded* >( *changeIt );
                     (*it)->applyCreateTriangleFunction(ea->triangleIndexArray);
                  }
                  break;
               }
            case core::topology::TRIANGLESREMOVED:
               {
                  if ((*it)->m_destroyTriangleFunc)
                  {
                     const TrianglesRemoved *er=static_cast< const TrianglesRemoved * >( *changeIt );
                     (*it)->applyDestroyTriangleFunction(er->getArray());
                  }
                  break;
               }
            case core::topology::TRIANGLESMOVED_REMOVING:
               {
                  if ((*it)->m_destroyTriangleFunc)
                  {
                     const TrianglesMoved_Removing *tm=static_cast< const TrianglesMoved_Removing* >( *changeIt );
                     (*it)->applyDestroyTriangleFunction(tm->trianglesAroundVertexMoved);
                  }
                  break;
               }
            case core::topology::TRIANGLESMOVED_ADDING:
               {
                  if ((*it)->m_createTriangleFunc)
                  {
                     const TrianglesMoved_Adding *tm=static_cast< const TrianglesMoved_Adding * >( *changeIt );
                     (*it)->applyCreateTriangleFunction(tm->trianglesAroundVertexMoved);
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
                  if ((*it)->m_createQuadFunc)
                  {
                     const QuadsAdded *ea=static_cast< const QuadsAdded* >( *changeIt );
                     (*it)->applyCreateQuadFunction(ea->quadIndexArray);
                  }
                  break;
               }
            case core::topology::QUADSREMOVED:
               {
                  if ((*it)->m_destroyQuadFunc)
                  {
                     const QuadsRemoved *er=static_cast< const QuadsRemoved * >( *changeIt );
                     (*it)->applyDestroyQuadFunction(er->getArray());
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
                  if ((*it)->m_createTetrahedronFunc)
                  {
                     const TetrahedraAdded *ea=static_cast< const TetrahedraAdded* >( *changeIt );
                     (*it)->applyCreateTetrahedronFunction(ea->tetrahedronIndexArray);
                  }
                  break;
               }
            case core::topology::TETRAHEDRAREMOVED:
               {
                  if ((*it)->m_destroyTetrahedronFunc)
                  {
                     const TetrahedraRemoved *er=static_cast< const TetrahedraRemoved * >( *changeIt );
                     (*it)->applyDestroyTetrahedronFunction(er->getArray());
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
                  if ((*it)->m_createHexahedronFunc)
                  {
                     const HexahedraAdded *ea=static_cast< const HexahedraAdded* >( *changeIt );
                     (*it)->applyCreateHexahedronFunction(ea->hexahedronIndexArray);
                  }
                  break;
               }
            case core::topology::HEXAHEDRAREMOVED:
               {
                  if ((*it)->m_destroyHexahedronFunc)
                  {
                     const HexahedraRemoved *er=static_cast< const HexahedraRemoved * >( *changeIt );
                     (*it)->applyDestroyHexahedronFunction(er->getArray());
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
      }

      m_changeList.endEdit();
      std::cout << "EdgeSetTopologyEngine::update() - end" << std::endl;
   }


}// namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYENGINE_CPP
