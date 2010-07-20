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
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_CPP
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_CPP
#include <sofa/component/topology/HexahedronSetTopologyEngine.h>
#include <sofa/component/topology/HexahedronSetTopologyContainer.h> // should not be include here!!

#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/HexahedronData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   HexahedronSetTopologyEngine::HexahedronSetTopologyEngine() : sofa::core::topology::TopologyEngine()
   {

   }

   void HexahedronSetTopologyEngine::init()
   {
      std::cout << "HexahedronSetTopologyEngine::init()" << std::endl;
      sofa::component::topology::PointSetTopologyContainer* topo_container;
      this->getContext()->get(topo_container);

      if (!topo_container)
      {
         std::cerr << "Error in HexahedronSetTopologyEngine: PointSetTopologyContainer not found." << std::endl;
         return;
      }

      this->m_changeList = topo_container->getChangeList();

      addInput(&m_changeList);

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
         addOutput((*it));

      std::cout << "HexahedronSetTopologyEngine::init() - end" << std::endl;
   }


   void HexahedronSetTopologyEngine::reinit()
   {
      this->update();
   }

   void HexahedronSetTopologyEngine::update()
   {
      std::cout << "HexahedronSetTopologyEngine::update()" << std::endl;
      cleanDirty();
      std::cout << "HexahedronSetTopologyEngine::update() - end" << std::endl;
   }


   void HexahedronSetTopologyEngine::handleTopologyChange()
   {
      std::cout << "HexahedronSetTopologyEngine::update()" << std::endl;
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (_iterator it=m_topologicalData.begin(); it!=m_topologicalData.end(); ++it)
      {
         for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
         {
            core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

            switch( changeType ) // See enum events in sofa::core::topology::Topology
            {
               // Events concerning Hexahedra
            case core::topology::HEXAHEDRAADDED:
               {
                  const HexahedraAdded *ta=static_cast< const HexahedraAdded * >( *changeIt );
                  (*it)->add( ta->getNbAddedHexahedra(), ta->hexahedronArray, ta->ancestorsList, ta->coefs );
                  break;
               }
            case core::topology::HEXAHEDRAREMOVED:
               {
                  const sofa::helper::vector<unsigned int> &tab = ( static_cast< const HexahedraRemoved *>( *changeIt ) )->getArray();
                  (*it)->remove( tab );
                  break;
               }
            case core::topology::HEXAHEDRARENUMBERING:
               {
                  /// FUNCTION MISSING HERE ///

                  break;
               }
            default:
               break;
            }

         }
      }

      m_changeList.endEdit();
      std::cout << "HexahedronSetTopologyEngine::update() - end" << std::endl;
   }


}// namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_CPP
