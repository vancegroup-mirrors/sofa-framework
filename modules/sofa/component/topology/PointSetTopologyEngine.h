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
#ifndef SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_H
#define SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_H

#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/list.h>
#include <sofa/component/topology/PointData.h>
#include <sofa/component/component.h>

namespace sofa
{
   namespace core { namespace topology { class TopologyChange; }}

namespace component
{

namespace topology
{


   using core::topology::BaseTopology;

   template< class T, class VecT = helper::vector<T> >
   class PointSetTopologyEngine : public sofa::core::topology::TopologyEngine
   {
   public:
      //SOFA_CLASS(PointSetTopologyEngine, sofa::core::topology::TopologyEngine);
      typedef PointData<T,VecT> t_topologicalData;

      typedef T value_type;
      typedef VecT container_type;

      typedef typename PointData<T,VecT>::t_createFunc t_createFunc;
      typedef typename PointData<T,VecT>::t_destroyFunc t_destroyFunc;
      typedef typename PointData<T,VecT>::t_createEdgeFunc t_createEdgeFunc;
      typedef typename PointData<T,VecT>::t_destroyEdgeFunc t_destroyEdgeFunc;
      typedef typename PointData<T,VecT>::t_createTriangleFunc t_createTriangleFunc;
      typedef typename PointData<T,VecT>::t_destroyTriangleFunc t_destroyTriangleFunc;
      typedef typename PointData<T,VecT>::t_createQuadFunc t_createQuadFunc;
      typedef typename PointData<T,VecT>::t_destroyQuadFunc t_destroyQuadFunc;
      typedef typename PointData<T,VecT>::t_createTetrahedronFunc t_createTetrahedronFunc;
      typedef typename PointData<T,VecT>::t_destroyTetrahedronFunc t_destroyTetrahedronFunc;
      typedef typename PointData<T,VecT>::t_createHexahedronFunc t_createHexahedronFunc;
      typedef typename PointData<T,VecT>::t_destroyHexahedronFunc t_destroyHexahedronFunc;

      PointSetTopologyEngine();
            
      PointSetTopologyEngine(PointData<T,VecT>* _topologicalData);

      PointSetTopologyEngine(PointData<T,VecT>* _topologicalData, sofa::core::topology::BaseMeshTopology* _topology);

      void init();

      void reinit();

      void update();

      void ApplyTopologyChanges();

      void registerTopology(sofa::core::topology::BaseMeshTopology* _topology);

      void registerTopologicalData(t_topologicalData *topologicalData) {m_topologicalData = topologicalData;}


//      void handleTopologyEvents( std::list< const core::topology::TopologyChange *>::const_iterator changeIt, std::list< const core::topology::TopologyChange *>::const_iterator &end );


      /// Creation function, called when adding elements.
      void setCreateFunction(t_createFunc createFunc);
      /// Destruction function, called when deleting elements.
      void setDestroyFunction(t_destroyFunc destroyFunc);

      /// Creation function, called when adding edges elements.
      void setCreateEdgeFunction(t_createEdgeFunc createEdgeFunc);
      /// Destruction function, called when removing edges elements.
      void setDestroyEdgeFunction(t_destroyEdgeFunc destroyEdgeFunc);

      /// Creation function, called when adding triangles elements.
      void setCreateTriangleFunction(t_createTriangleFunc createTriangleFunc);
      /// Destruction function, called when removing triangles elements.
      void setDestroyTriangleFunction(t_destroyTriangleFunc destroyTriangleFunc);

      /// Creation function, called when adding quads elements.
      void setCreateQuadFunction(t_createQuadFunc createQuadFunc);
      /// Destruction function, called when removing quads elements.
      void setDestroyQuadFunction(t_destroyQuadFunc destroyQuadFunc);

      /// Creation function, called when adding tetrahedra elements.
      void setCreateTetrahedronFunction(t_createTetrahedronFunc createTetrahedronFunc);
      /// Destruction function, called when removing tetrahedra elements.
      void setDestroyTetrahedronFunction(t_destroyTetrahedronFunc destroyTetrahedronFunc);

      /// Creation function, called when adding hexahedra elements.
      void setCreateHexahedronFunction(t_createHexahedronFunc createHexahedronFunc);
      /// Destruction function, called when removing hexahedra elements.
      void setDestroyHexahedronFunction(t_destroyHexahedronFunc destroyHexahedronFunc);

      /// Creation function, called when adding parameter to those elements.
      void setDestroyParameter( void* destroyParam );
      /// Destruction function, called when removing parameter to those elements.
      void setCreateParameter( void* createParam );

      /// Function to link DataEngine with Data array from topology
      void linkToPointDataArray();
      void linkToEdgeDataArray();
      void linkToTriangleDataArray();
      void linkToQuadDataArray();
      void linkToTetrahedronDataArray();
      void linkToHexahedronDataArray();

   protected:
      t_topologicalData* m_topologicalData;
      sofa::core::topology::TopologyContainer* m_topology;

   public:
      bool m_pointsLinked;
      bool m_edgesLinked;
      bool m_trianglesLinked;
      bool m_quadsLinked;
      bool m_tetrahedraLinked;
      bool m_hexahedraLinked;

   };


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_POINTSETTOPOLOGYENGINE_H
