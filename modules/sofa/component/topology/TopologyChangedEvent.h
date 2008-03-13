
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYCHANGEDEVENT_H
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYCHANGEDEVENT_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/componentmodel/topology/BaseTopology.h>

namespace sofa
{

namespace component
{

namespace topology
{

/**
	@author H. Delingette
	*/
	class TopologyChangedEvent : public sofa::core::objectmodel::Event
	{
	protected:
		core::componentmodel::topology::BaseTopology *topology;

	public:
		TopologyChangedEvent( core::componentmodel::topology::BaseTopology *_topology) : topology(_topology) {
		}

		~TopologyChangedEvent(){}

		core::componentmodel::topology::BaseTopology *getTopology() const{
			return topology;
		}
	};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
