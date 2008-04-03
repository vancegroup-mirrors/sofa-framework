#include "EdgeRemoveContact.h"
#include <sofa/component/topology/EdgeSetTopology.h>
#include <set>

namespace sofa
{

namespace component
{

namespace collision
{

void EdgeRemoveContact::createResponse(sofa::core::objectmodel::BaseContext* /*group*/)
{
    sofa::core::objectmodel::BaseContext* context = model2->getContext();
    topology::EdgeSetTopology< Vec3Types >* topology = dynamic_cast< EdgeSetTopology< Vec3Types >* >( context->getMainTopology() );
    //simulation::tree::GNode * node = dynamic_cast<simulation::tree::GNode *>(context);
    //assert( node != NULL ); // otherwise, collision model has no behavior model corresponding...
    //while ( node->mechanicalMapping )
    //{
    //    node = node->parent;
    //}
    core::componentmodel::behavior::MechanicalState<Vec3Types>* mechState = dynamic_cast< core::componentmodel::behavior::MechanicalState< Vec3Types >* >(context->getMechanicalState());
    if (topology != NULL && mechState != NULL)
    {
        
        for (unsigned int cindex=0; cindex<collisions.size(); cindex++)
        {
            sofa::helper::vector< unsigned int > toBeCut;
            unsigned int pindex = (unsigned int) collisions[cindex].elem.second.getIndex();
            sofa::helper::vector< unsigned int > edgeVertexShell = topology->getEdgeSetTopologyContainer()->getEdgeVertexShell( pindex );
            for (unsigned int i = 0; i < edgeVertexShell.size(); ++i)
            {
                toBeCut.push_back( edgeVertexShell[i] );
            }
            if (!toBeCut.empty())
            {
		std::cout << "Removing "<<toBeCut.size()<<" / "<<topology->getEdgeSetTopologyContainer()->getEdgeArray().size()<<" edges."<<std::endl;
                topology->getEdgeSetTopologyAlgorithms()->removeEdges( toBeCut );
            }
        }
    }
}

SOFA_DECL_CLASS(EdgeRemoveContact)

Creator<core::componentmodel::collision::Contact::Factory, EdgeRemoveContact > EdgeRemoveContactClass("EdgeRemove",true);

} // namespace collision

} // namespace component

} // namespace sofa
