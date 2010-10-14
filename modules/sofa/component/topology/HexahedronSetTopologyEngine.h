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
#ifndef SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_H
#define SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_H

#include <sofa/core/topology/BaseTopology.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/component/topology/HexahedronData.h>

namespace sofa
{

namespace core { namespace topology { class TopologyChange; }}

namespace component
{

namespace topology
{
   using core::topology::BaseTopology;

   template< class T, class Alloc = helper::CPUMemoryManager<T> >
   class HexahedronSetTopologyEngine : public sofa::core::topology::TopologyEngine
   {
   public:
      //SOFA_CLASS(HexahedronSetTopologyEngine, sofa::core::topology::TopologyEngine);
      typedef HexahedronData<T,Alloc> t_topologicalData;

      typedef typename t_topologicalData::t_createFunc t_createFunc;
      typedef typename t_topologicalData::t_destroyFunc t_destroyFunc;

      HexahedronSetTopologyEngine();

      HexahedronSetTopologyEngine(t_topologicalData* _topologicalData);

      HexahedronSetTopologyEngine(t_topologicalData* _topologicalData, sofa::core::topology::BaseMeshTopology* _topology);


      virtual ~HexahedronSetTopologyEngine() {}

      virtual void init();

      virtual void reinit();

      virtual void update();

      void ApplyTopologyChanges();

      void registerTopology(sofa::core::topology::BaseMeshTopology* _topology);

      void registerTopologicalData(t_topologicalData *topologicalData) {m_topologicalData = topologicalData;}

      /// Creation function, called when adding elements.
      void setCreateFunction(t_createFunc createFunc);
      /// Destruction function, called when deleting elements.
      void setDestroyFunction(t_destroyFunc destroyFunc);

      /// Creation function, called when adding parameter to those elements.
      void setDestroyParameter( void* destroyParam );
      /// Destruction function, called when removing parameter to those elements.
      void setCreateParameter( void* createParam );

      /// Function to link DataEngine with Data array from topology
      void linkToHexahedronDataArray();

   protected:
      t_topologicalData* m_topologicalData;
      sofa::core::topology::TopologyContainer* m_topology;

   public:
      bool m_hexahedraLinked;
   };


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_HEXAHEDRONSETTOPOLOGYENGINE_H
