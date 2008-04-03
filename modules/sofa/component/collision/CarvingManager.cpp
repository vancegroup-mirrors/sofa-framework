#include "CarvingManager.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/tree/AnimateBeginEvent.h>
#include <sofa/simulation/tree/AnimateEndEvent.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(CarvingManager)

int CarvingManagerClass = core::RegisterObject("Manager handling carving operations between a SphereModel and a TriangleSetModel relying on a TetrahedronSetTopology")
.add< CarvingManager >()
;


CarvingManager::CarvingManager()
: f_modelTool( initData(&f_modelTool, "modelTool", "SharpLineModel path"))
, f_modelSurface( initData(&f_modelSurface, "modelSurface", "TriangleSetModel path"))
, active( initData(&active, false, "active", "Activate this object. Note that this can be dynamically controlled by using a key") )
, keyEvent( initData(&keyEvent, '1', "key", "key to press to activate this object until the key is released") )
, keySwitchEvent( initData(&keySwitchEvent, '4', "keySwitch", "key to activate this object until the key is pressed again") )
, modelTool(NULL)
, modelSurface(NULL)
, intersectionMethod(NULL)
, detectionNP(NULL)
{
    this->f_listening.setValue(true);
}

CarvingManager::~CarvingManager()
{
}

void CarvingManager::init()
{
    if (f_modelTool.getValue().empty())
        modelTool = getContext()->get<ToolModel>(core::objectmodel::BaseContext::SearchDown);
    else
        modelTool = getContext()->get<ToolModel>(f_modelTool.getValue());
    if (f_modelSurface.getValue().empty())
    {
        // we look for a TriangleSetModel relying on a TetrahedronSetTopology.
        //modelSurface = getContext()->get<TriangleSetModel>(core::objectmodel::BaseContext::SearchDown);
        std::vector<TriangleSetModel*> models;
        getContext()->get<TriangleSetModel>(&models, core::objectmodel::BaseContext::SearchRoot);
        for (unsigned int i=0;i<models.size();++i)
        {
            TriangleSetModel* m = models[i];
            sofa::core::componentmodel::topology::BaseTopology* bt = m->getTopology();
            if (bt == NULL) continue;
            sofa::core::componentmodel::topology::TopologyContainer *container=bt->getTopologyContainer();
            //sofa::component::topology::TriangleSetTopologyContainer *tstc= dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer *>(container);
            sofa::component::topology::TetrahedronSetTopologyContainer *testc= dynamic_cast<sofa::component::topology::TetrahedronSetTopologyContainer *>(container);
            //if (tstc == NULL) continue;
            if (testc == NULL) continue;
            modelSurface = m; // we found a good object
            break;
        }
    }
    else
        modelSurface = getContext()->get<TriangleSetModel>(f_modelSurface.getValue());
    intersectionMethod = getContext()->get<core::componentmodel::collision::Intersection>();
    detectionNP = getContext()->get<core::componentmodel::collision::NarrowPhaseDetection>();
    bool error = false;
    if (modelTool == NULL) { std::cerr << "CarvingManager: modelTool not found\n"; error = true; }
    if (modelSurface == NULL) { std::cerr << "CarvingManager: modelSurface not found\n"; error = true; }
    if (intersectionMethod == NULL) { std::cerr << "CarvingManager: intersectionMethod not found\n"; error = true; }
    if (detectionNP == NULL) { std::cerr << "CarvingManager: NarrowPhaseDetection not found\n"; error = true; }
    if (!error)
        std::cout << "CarvingManager: init OK." << std::endl;
}

void CarvingManager::reset()
{
}

void CarvingManager::doCarve()
{
    if (modelTool==NULL || modelSurface==NULL || intersectionMethod == NULL || detectionNP == NULL) return;
    sofa::component::topology::TetrahedronSetTopology<DataTypes>* topo = dynamic_cast<sofa::component::topology::TetrahedronSetTopology<DataTypes> *>(modelSurface->getTopology());
    if (topo == NULL) return;
    sofa::component::topology::TetrahedronSetTopologyContainer *tstc= topo->getTetrahedronSetTopologyContainer();
    if (tstc == NULL) return;
    
    // do collision detection
    //std::cout << "CarvingManager: build bounding trees" << std::endl;
    {
        const bool continuous = intersectionMethod->useContinuous();
        const double dt       = getContext()->getDt();
        const int depth = 6;

        if (continuous)
            modelTool->computeContinuousBoundingTree(dt, depth);
        else
            modelTool->computeBoundingTree(depth);

        if (continuous)
            modelSurface->computeContinuousBoundingTree(dt, depth);
        else
            modelSurface->computeBoundingTree(depth);
    }
    
    sofa::helper::vector<std::pair<core::CollisionModel*, core::CollisionModel*> > vectCMPair;
    //vectCMPair = broadPhaseDetection->getCollisionModelPairs();

    //vectCMPair.push_back(std::make_pair(modelSurface,modelTool));
    vectCMPair.push_back(std::make_pair(modelSurface->getFirst(),modelTool->getFirst()));
    
    //std::cout << "CarvingManager: narrow phase" << std::endl;
    detectionNP->setInstance(this);
    detectionNP->setIntersectionMethod(intersectionMethod);
    detectionNP->beginNarrowPhase();
    detectionNP->addCollisionPairs(vectCMPair);
    detectionNP->endNarrowPhase();
    
    const core::componentmodel::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = detectionNP->getDetectionOutputs();
    
    //std::cout << "CarvingManager: process contacts" << std::endl;

    const ContactVector* contacts = NULL;
    core::componentmodel::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); //find(std::make_pair(modelSurface,modelTool));
    if (it != detectionOutputs.end())
    {
	std::cout << "Contacts available..." << std::endl;
        contacts = dynamic_cast<const ContactVector*>(it->second);
    }
    unsigned int ncontacts = 0;
    if (contacts != NULL)
    {
        ncontacts = contacts->size();
        std::cout << contacts->size() << " contacts detected." << std::endl;
    }
    std::set<unsigned> elemsToRemove;
    for (unsigned int j=0; j < ncontacts; ++j)
    {
        const ContactVector::value_type& c = (*contacts)[j];
        int triangleIdx = (c.elem.first.getCollisionModel()==modelSurface ? c.elem.first.getIndex():c.elem.second.getIndex());
        // convert from local collision model indices to global topology ones
        triangleIdx = modelSurface->getLoc2GlobVec()[triangleIdx];
        const sofa::helper::vector< unsigned int >& tetras = tstc->getTetrahedronTriangleShell(triangleIdx);
        elemsToRemove.insert(tetras.begin(), tetras.end());
    }
    if (!elemsToRemove.empty())
    {
        std::cout << elemsToRemove.size() << " tetrahedra to remove"<<std::endl;
        sofa::helper::vector< unsigned int > tetrahedra;
        for (std::set<unsigned>::const_iterator it = elemsToRemove.begin(), itend = elemsToRemove.end(); it != itend; ++it)
            tetrahedra.push_back(*it);
        topo->getTetrahedronSetTopologyAlgorithms()->removeTetrahedra(tetrahedra);
    }
    detectionNP->setInstance(NULL);
    //std::cout << "CarvingManager: carve done" << std::endl;
}

void CarvingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ev->getKey() == keyEvent.getValue())
        {
            active.setValue(true);
        }
        else if (ev->getKey() == keySwitchEvent.getValue())
        {
            active.setValue(!active.getValue());
        }
    }
    else if (sofa::core::objectmodel::KeyreleasedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeyreleasedEvent*>(event))
    {
        if (ev->getKey() == keyEvent.getValue())
        {
            active.setValue(false);
        }
    }
    else if (/* simulation::tree::AnimateEndEvent* ev = */ dynamic_cast<simulation::tree::AnimateEndEvent*>(event))
    {
        if (active.getValue())
            doCarve();
    }
}

} // namespace collision

} // namespace component

} // namespace sofa
