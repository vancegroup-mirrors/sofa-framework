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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYENGINE_CPP
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYENGINE_CPP
#include <sofa/component/topology/TriangleSetTopologyEngine.h>
#include <sofa/component/topology/TriangleSetTopologyContainer.h> // should not be include here!!

#include <sofa/component/topology/TriangleSetTopologyChange.h>
#include <sofa/component/topology/QuadSetTopologyChange.h>
#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/TriangleData.inl>


namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   TriangleSetTopologyEngine::TriangleSetTopologyEngine() : sofa::core::topology::TopologyEngine()
   {

   }

   void TriangleSetTopologyEngine::init()
   {
      std::cout << "TriangleSetTopologyEngine::init()" << std::endl;
      sofa::component::topology::PointSetTopologyContainer* topo_container;
      this->getContext()->get(topo_container);

      if (!topo_container)
      {
         std::cerr << "Error in TriangleSetTopologyEngine: PointSetTopologyContainer not found." << std::endl;
         return;
      }

      this->m_changeList = topo_container->getChangeList();

      addInput(&m_changeList);

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
         addOutput((*it));

      std::cout << "TriangleSetTopologyEngine::init() - end" << std::endl;
   }


   void TriangleSetTopologyEngine::reinit()
   {
      this->update();
   }

   void TriangleSetTopologyEngine::update()
   {
      std::cout << "TriangleSetTopologyEngine::update()" << std::endl;
      cleanDirty();
      std::cout << "TriangleSetTopologyEngine::update() - end" << std::endl;
   }


   void TriangleSetTopologyEngine::handleTopologyChange()
   {
      std::cout << "TriangleSetTopologyEngine::update()" << std::endl;
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
      {
         for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
         {
            core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

            switch( changeType ) // See enum events in sofa::core::topology::Topology
            {
               // Events concerning Triangles
            case core::topology::TRIANGLESADDED:
               {
                  const TrianglesAdded *ta = static_cast< const TrianglesAdded * >( *changeIt );
                  (*it)->add( ta->getNbAddedTriangles(), ta->triangleArray, ta->ancestorsList, ta->coefs );
                  break;
               }
            case core::topology::TRIANGLESREMOVED:
               {
                  const sofa::helper::vector<unsigned int> &tab = ( static_cast< const TrianglesRemoved *>( *changeIt ) )->getArray();
                  (*it)->remove( tab );
                  break;
               }
            case core::topology::TRIANGLESMOVED_REMOVING:
               {
                  const sofa::helper::vector< unsigned int >& triList = ( static_cast< const TrianglesMoved_Removing *>( *changeIt ) )->trianglesAroundVertexMoved;

                  (*it)->remove( triList );
                  break;
               }
            case core::topology::TRIANGLESMOVED_ADDING:
               {
                  const sofa::helper::vector< unsigned int >& triList = ( static_cast< const TrianglesMoved_Adding *>( *changeIt ) )->trianglesAroundVertexMoved;
                  const sofa::helper::vector< Triangle >& triArray = ( static_cast< const TrianglesMoved_Adding *>( *changeIt ) )->triangleArray2Moved;

                  // Recompute data
                  sofa::helper::vector< sofa::helper::vector< unsigned int > > ancestors;
                  sofa::helper::vector< sofa::helper::vector< double > > coefs;
                  coefs.resize(triList.size());
                  ancestors.resize(triList.size());

                  for (unsigned int i = 0; i <triList.size(); i++)
                  {
                     ancestors[i].push_back(1);
                     ancestors[i].push_back(triList[i]);
                  }

                  (*it)->add( triList.size(), triArray, ancestors, coefs);
                  break;
               }
            case core::topology::TRIANGLESRENUMBERING:
               {
                  /// FUNCTION MISSING HERE ///


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
                  sout << "QUADSRENUMBERING topological change is not implemented in TriangleSetTopologyEngine" << sendl;
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
                  sout << "TETRAHEDRARENUMBERING topological change is not implemented in TriangleSetTopologyEngine" << sendl;
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
                  sout << "HEXAHEDRARENUMBERING topological change is not implemented in TriangleSetTopologyEngine" << sendl;
#endif
                  break;
               }
            default:
               break;
            }

         }
      }
      m_changeList.endEdit();
      std::cout << "TriangleSetTopologyEngine::update() - end" << std::endl;
   }


}// namespace topology

} // namespace component

} // namespace sofa



#endif // SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYENGINE_CPP
