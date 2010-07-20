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
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_CPP
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_CPP
#include <sofa/component/topology/TetrahedronSetTopologyEngine.h>
#include <sofa/component/topology/TetrahedronSetTopologyContainer.h> // should not be include here!!

#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/TetrahedronData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   TetrahedronSetTopologyEngine::TetrahedronSetTopologyEngine() : sofa::core::topology::TopologyEngine()
   {
   }

   void TetrahedronSetTopologyEngine::init()
   {
      std::cout << "TetrahedronSetTopologyEngine::init()" << std::endl;
      sofa::component::topology::PointSetTopologyContainer* topo_container;
      this->getContext()->get(topo_container);

      if (!topo_container)
      {
         std::cerr << "Error in TetrahedronSetTopologyEngine: PointSetTopologyContainer not found." << std::endl;
         return;
      }

      this->m_changeList = topo_container->getChangeList();

      addInput(&m_changeList);

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
         addOutput((*it));

      std::cout << "TetrahedronSetTopologyEngine::init() - end" << std::endl;
   }


   void TetrahedronSetTopologyEngine::reinit()
   {
      this->update();
   }

   void TetrahedronSetTopologyEngine::update()
   {
      std::cout << "TetrahedronSetTopologyEngine::update()" << std::endl;
      cleanDirty();
      std::cout << "TetrahedronSetTopologyEngine::update() - end" << std::endl;
   }


   void TetrahedronSetTopologyEngine::handleTopologyChange()
   {
      std::cout << "TetrahedronSetTopologyEngine::update()" << std::endl;
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
      {
         for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
         {
            core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

            switch( changeType ) // See enum events in sofa::core::topology::Topology
            {
               // Events concerning Tetrahedra
            case core::topology::TETRAHEDRAADDED:
               {
                  const TetrahedraAdded *ta = static_cast< const TetrahedraAdded * >( *changeIt );
                  (*it)->add( ta->getNbAddedTetrahedra(), ta->tetrahedronArray, ta->ancestorsList, ta->coefs );
                  break;
               }
            case core::topology::TETRAHEDRAREMOVED:
               {
                  const sofa::helper::vector<unsigned int> &tab = ( static_cast< const TetrahedraRemoved *>( *changeIt ) )->getArray();
                  (*it)->remove( tab );
                  break;
               }
            case core::topology::TETRAHEDRARENUMBERING:
               {
                  /// FUNCTION MISSING HERE ///


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
                  sout << "HEXAHEDRARENUMBERING topological change is not implemented in TetrahedronSetTopologyEngine" << sendl;
#endif
                  break;
               }
            default:
               break;
            }

         }
      }

      m_changeList.endEdit();
      std::cout << "TetrahedronSetTopologyEngine::update() - end" << std::endl;
   }


}// namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_CPP
