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
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_INL
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_INL
#include <sofa/component/topology/TetrahedronSetTopologyEngine.h>

#include <sofa/component/topology/TetrahedronSetTopologyChange.h>
#include <sofa/component/topology/HexahedronSetTopologyChange.h>

#include <sofa/component/topology/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/HexahedronSetTopologyContainer.h>

#include <sofa/component/topology/TetrahedronData.inl>

namespace sofa
{
namespace component
{
namespace topology
{

   using namespace sofa::component::topology;

   template <typename T, typename Alloc>
   TetrahedronSetTopologyEngine<T,Alloc>::TetrahedronSetTopologyEngine() : sofa::core::topology::TopologyEngine(),
   m_topology(NULL),
   m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }


   template <typename T, typename Alloc>
   TetrahedronSetTopologyEngine<T,Alloc>::TetrahedronSetTopologyEngine(TetrahedronData<T, Alloc> *_topologicalData) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      this->init();
   }

   template <typename T, typename Alloc>
   TetrahedronSetTopologyEngine<T,Alloc>::TetrahedronSetTopologyEngine(TetrahedronData<T, Alloc> *_topologicalData, sofa::core::topology::BaseMeshTopology *_topology) :
   m_topologicalData(_topologicalData),
   m_topology(NULL),
   m_tetrahedraLinked(false), m_hexahedraLinked(false)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;

      this->init();
   }

   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::init()
   {
      this->addInput(&m_changeList);
      this->addOutput(m_topologicalData);
      //sofa::core::topology::TopologyEngine::init();

      if (m_topology)
         m_topology->addTopologyEngine(this);
   }


   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::reinit()
   {
      this->update();
   }

   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::update()
   {
      std::cout << "TetrahedronSetTopologyEngine::update()" << std::endl;
      std::cout<< "Number of topological changes: " << m_changeList.getValue().size() << std::endl;
      this->ApplyTopologyChanges();
      std::cout << "TetrahedronSetTopologyEngine::update() - end" << std::endl;
   }

   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::registerTopology(sofa::core::topology::BaseMeshTopology *_topology)
   {
      m_topology =  dynamic_cast<sofa::core::topology::TopologyContainer*>(_topology);

      if (m_topology == NULL)
         std::cerr <<"Error: Topology is not dynamic" << std::endl;
   }


   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::ApplyTopologyChanges()
   {
      sofa::helper::list<const core::topology::TopologyChange *>::iterator changeIt;
      sofa::helper::list<const core::topology::TopologyChange *>& _changeList = *m_changeList.beginEdit();

      for (changeIt=_changeList.begin(); changeIt!=_changeList.end(); ++changeIt)
      {
         core::topology::TopologyChangeType changeType = (*changeIt)->getChangeType();

         switch( changeType ) // See enum events in sofa::core::topology::Topology
         {
            // Events concerning Tetrahedra
         case core::topology::TETRAHEDRAADDED:
            {
               const TetrahedraAdded *ta = static_cast< const TetrahedraAdded * >( *changeIt );
               m_topologicalData->add( ta->getNbAddedTetrahedra(), ta->tetrahedronArray, ta->ancestorsList, ta->coefs );
               break;
            }
         case core::topology::TETRAHEDRAREMOVED:
            {
               const sofa::helper::vector<unsigned int> &tab = ( static_cast< const TetrahedraRemoved *>( *changeIt ) )->getArray();
               m_topologicalData->remove( tab );
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
               sout << "HEXAHEDRARENUMBERING topological change is not implemented in TetrahedronSetTopologyEngine" << sendl;
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
   void TetrahedronSetTopologyEngine<T,Alloc>::setCreateFunction(t_createFunc createFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To EdgeData
         m_topologicalData->setCreateFunction(createFunc);

         if (!m_tetrahedraLinked && m_topology) // No link
            this->linkToTetrahedronDataArray();
      }
   }

   /// Destruction function, called when deleting elements.
   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::setDestroyFunction(t_destroyFunc destroyFunc)
   {
      if (m_topologicalData)
      {
         // Transfert pointer to function To EdgeData
         m_topologicalData->setDestroyFunction(destroyFunc);

         if (!m_tetrahedraLinked && m_topology) // No link
            this->linkToTetrahedronDataArray();
      }
   }


   /// Creation function, called when adding hexahedra elements.
   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::setCreateHexahedronFunction(t_createHexahedronFunc createHexahedronFunc)
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
   void TetrahedronSetTopologyEngine<T,Alloc>::setDestroyHexahedronFunction(t_destroyHexahedronFunc destroyHexahedronFunc)
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
   void TetrahedronSetTopologyEngine<T,Alloc>::setDestroyParameter( void* destroyParam )
   {
      if (m_topologicalData)
         m_topologicalData->setDestroyParameter(destroyParam);
   }

   /// Destruction function, called when removing parameter to those elements.
   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::setCreateParameter( void* createParam )
   {
      if (m_topologicalData)
         m_topologicalData->setCreateParameter(createParam);
   }



   template <typename T, typename Alloc>
   void TetrahedronSetTopologyEngine<T,Alloc>::linkToTetrahedronDataArray()
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
   void TetrahedronSetTopologyEngine<T,Alloc>::linkToHexahedronDataArray()
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


#endif // SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYENGINE_CPP
