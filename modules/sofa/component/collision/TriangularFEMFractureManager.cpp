#include "TriangularFEMFractureManager.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#include <sofa/helper/gl/template.h>
#include <sofa/component/forcefield/TriangularFEMForceField.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(TriangularFEMFractureManager)

int TriangularFEMFractureManagerClass = core::RegisterObject("Manager handling fracture operations between a SharpLineModel and a TriangleModel")
.add< TriangularFEMFractureManager >()
;


TriangularFEMFractureManager::TriangularFEMFractureManager()
:f_femName(initData(&f_femName,std::string(" "),"femName","The TriangularFEMForceField name that is targeted by this fracture manager"))
{
	this->f_listening.setValue(true);
}

TriangularFEMFractureManager::~TriangularFEMFractureManager()
{
}

void TriangularFEMFractureManager::init()
{
	std::vector<core::componentmodel::behavior::BaseForceField*> ffs;
	getContext()->get<core::componentmodel::behavior::BaseForceField>(&ffs);
	
	for (unsigned int i=0;i<ffs.size();++i)
	{
		if (ffs[i]->getName() == f_femName.getValue())
		{
			std::cout << "FEM Found\n";
			femModel = ffs[i];
			break;
		}
	}
}

void TriangularFEMFractureManager::reset()
{
    
}

void TriangularFEMFractureManager::doFracture()
{
	topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( getContext()->getMainTopology() );

	int fracturedEdge = femModel->getFracturedEdge();

	if (fracturedEdge != -1)
	{
		sofa::helper::vector< unsigned int > triangleEdgeShell = tsp->getTriangleSetTopologyContainer()->getTriangleEdgeShell(fracturedEdge);
		if (triangleEdgeShell.size() != 1)
		{
		//	const sofa::helper::vector< Edge> &edgeArray=tsp->getTriangleSetTopologyContainer()->getEdgeArray();
		//	std::cout << "InciseAlongEdge " << edgeArray[fracturedEdge].first << " " << edgeArray[fracturedEdge].second << "\n";
			tsp->getTriangleSetTopologyAlgorithms()->InciseAlongEdge(fracturedEdge);
		//	std::cout << "Incision is over\n";
		}	
	}
}

void TriangularFEMFractureManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<simulation::tree::AnimateBeginEvent*>(event))
    {
    }
    if (dynamic_cast<simulation::tree::AnimateEndEvent*>(event))
    {
        doFracture();
    }
}  

} // namespace collision

} // namespace component

} // namespace sofa
