#include "CuttingManager.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(CuttingManager)

int CuttingManagerClass = core::RegisterObject("Manager handling cutting operations between a SharpLineModel and a TriangleSetModel")
.add< CuttingManager >()
;


CuttingManager::CuttingManager()
: f_modelTool( initData(&f_modelTool, "modelTool", "SharpLineModel path"))
, f_modelSurface( initData(&f_modelSurface, "modelSurface", "TriangleSetModel path"))
, f_minDistance( initData(&f_minDistance, (Real)0.01f, "minDistance", "minimum distance between cutting steps"))
, f_maxDistance( initData(&f_maxDistance, (Real)1.00f, "maxDistance", "maximum distance between cutting steps"))
, f_edgeDistance( initData(&f_edgeDistance, (Real)0.1f, "edgeDistance", "minimum distance from an edge of a triangle (in terms of barycentric coordinates)"))
, modelTool(NULL)
, modelSurface(NULL)
, intersectionMethod(NULL)
, detectionNP(NULL)
{
    this->f_listening.setValue(true);
}

CuttingManager::~CuttingManager()
{
}

void CuttingManager::init()
{
    if (f_modelTool.getValue().empty())
        modelTool = getContext()->get<SharpLineModel>(core::objectmodel::BaseContext::SearchDown);
    else
        modelTool = getContext()->get<SharpLineModel>(f_modelTool.getValue());
    if (f_modelSurface.getValue().empty())
    {
        // we look for a cuttable TriangleSetModel.
        // this currently means a model with a TriangleSetTopology but not a TetrahedronSetTopology.
        //modelSurface = getContext()->get<TriangleSetModel>(core::objectmodel::BaseContext::SearchDown);
        std::vector<TriangleSetModel*> models;
        getContext()->get<TriangleSetModel>(&models, core::objectmodel::BaseContext::SearchRoot);
        for (unsigned int i=0;i<models.size();++i)
        {
            TriangleSetModel* m = models[i];
            sofa::core::componentmodel::topology::BaseTopology* bt = m->getTopology();
            if (bt == NULL) continue;
            sofa::core::componentmodel::topology::TopologyContainer *container=bt->getTopologyContainer();
            sofa::component::topology::TriangleSetTopologyContainer *tstc= dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer *>(container);
            sofa::component::topology::TetrahedronSetTopologyContainer *testc= dynamic_cast<sofa::component::topology::TetrahedronSetTopologyContainer *>(container);
            if (tstc == NULL) continue;
            if (testc != NULL) continue;
            modelSurface = m; // we found a cuttable object
            break;
        }
    }
    else
        modelSurface = getContext()->get<TriangleSetModel>(f_modelSurface.getValue());
    intersectionMethod = getContext()->get<core::componentmodel::collision::Intersection>();
    detectionNP = getContext()->get<core::componentmodel::collision::NarrowPhaseDetection>();
    bool error = false;
    if (modelTool == NULL) { std::cerr << "CuttingManager: modelTool not found\n"; error = true; }
    if (modelSurface == NULL) { std::cerr << "CuttingManager: modelSurface not found\n"; error = true; }
    if (intersectionMethod == NULL) { std::cerr << "CuttingManager: intersectionMethod not found\n"; error = true; }
    if (detectionNP == NULL) { std::cerr << "CuttingManager: NarrowPhaseDetection not found\n"; error = true; }
    if (!error)
        std::cout << "CuttingManager: init OK." << std::endl;
}

void CuttingManager::reset()
{
    cuttingPoints.clear();
}

void CuttingManager::doCut()
{
    if (modelTool==NULL || modelSurface==NULL || intersectionMethod == NULL || detectionNP == NULL) return;
    sofa::core::componentmodel::topology::BaseTopology* bt = dynamic_cast<sofa::core::componentmodel::topology::BaseTopology *>(modelSurface->getContext()->getMainTopology());
    if (bt == NULL) return;
    sofa::core::componentmodel::topology::TopologyContainer *container=bt->getTopologyContainer();
    sofa::component::topology::TriangleSetTopologyContainer *tstc= dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer *>(container);
    if (tstc == NULL) return;
    
    // do collision detection
    //std::cout << "CuttingManager: build bounding trees" << std::endl;
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
    
    //std::cout << "CuttingManager: narrow phase" << std::endl;
    detectionNP->setInstance(this);
    detectionNP->setIntersectionMethod(intersectionMethod);
    detectionNP->beginNarrowPhase();
    detectionNP->addCollisionPairs(vectCMPair);
    detectionNP->endNarrowPhase();
    
    const core::componentmodel::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = detectionNP->getDetectionOutputs();
    
    //std::cout << "CuttingManager: process contacts" << std::endl;

    const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>* contacts = NULL;
    core::componentmodel::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); //find(std::make_pair(modelSurface,modelTool));
    if (it != detectionOutputs.end())
        contacts = dynamic_cast<const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>*>(it->second);
    topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( modelSurface->getContext()->getMainTopology() );
    unsigned int ncontacts = 0;
    if (contacts != NULL)
    {
        ncontacts = contacts->size();
        std::cout << contacts->size() << " contacts detected." << std::endl;
    }
    
    const Real edgeDistance = f_edgeDistance.getValue();

    if (ncontacts==2 && cuttingPoints.empty() && (*contacts)[0].elem.first.getIndex() != (*contacts)[1].elem.first.getIndex() && ((*contacts)[0].point[0]-(*contacts)[1].point[0]).norm() > f_minDistance.getValue())
    {
        const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>::value_type& ca = (*contacts)[0];
        const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>::value_type& cb = (*contacts)[1];
        
        Coord a = ca.point[0];
        Coord b = cb.point[0];
        unsigned int ind_ta = ca.elem.first.getIndex();
        unsigned int ind_tb = cb.elem.first.getIndex();
        // Check if we are not too close to an edge of the triangle
        sofa::helper::vector< double > baryCoefs = tsp->getTriangleSetGeometryAlgorithms()->computeTriangleBarycoefs( b, ind_tb );
        if (baryCoefs.size() == 3 && (baryCoefs[0]<edgeDistance/2 || baryCoefs[1]<edgeDistance/2 || baryCoefs[2]<edgeDistance/2))
        {
            std::cout << "Too close to an edge" << std::endl;
            return;
        }
        baryCoefs = tsp->getTriangleSetGeometryAlgorithms()->computeTriangleBarycoefs( a, ind_ta );
        if (baryCoefs.size() == 3 && (baryCoefs[0]<edgeDistance/2 || baryCoefs[1]<edgeDistance/2 || baryCoefs[2]<edgeDistance/2))
        {
            std::cout << "Too close to an edge" << std::endl;
            return;
        }
        // first cut
        std::cout << "First cut"<<std::endl;
        
        unsigned int a_last = 0;
        sofa::helper::vector< unsigned int > a_p12_last;
        sofa::helper::vector< unsigned int > a_i123_last;

        unsigned int b_last = 0;
        sofa::helper::vector< unsigned int > b_p12_last;
        sofa::helper::vector< unsigned int > b_i123_last;
        
        sofa::helper::vector< sofa::helper::vector<unsigned int> > new_points;
        sofa::helper::vector< sofa::helper::vector<unsigned int> > closest_vertices;
        bool is_first_cut = true;
        bool is_fully_cut = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongPointsList(is_first_cut, a, b, ind_ta, ind_tb, 
                                                                                           a_last, a_p12_last, a_i123_last,
                                                                                           b_last, b_p12_last, b_i123_last, new_points, closest_vertices);
        is_fully_cut = false;
        
        if (b_p12_last.size() == 2 && b_i123_last.size() == 3)
        {
            CuttingPoint* cp = new CuttingPoint;
            modelSurface->getContext()->addObject(cp);
            cp->lastPos = a;
            cp->pos = b;
            cp->newPos = b;
            cp->toolElemIndex = cb.elem.second.getIndex();
            cp->points.resize(6);
            cp->points[0] = b_last;
            cp->points[1] = b_p12_last[0];
            cp->points[2] = b_p12_last[1];
            cp->points[3] = b_i123_last[0];
            cp->points[4] = b_i123_last[1];
            cp->points[5] = b_i123_last[2];
            cp->init();
            //cp->pos = (*cp->mstate->getX())[cp->points[0]];
            cuttingPoints.push_back(cp);
        }
        if (a_p12_last.size() == 2 && a_i123_last.size() == 3)
        {
            CuttingPoint* cp = new CuttingPoint;
            modelSurface->getContext()->addObject(cp);
            cp->lastPos = b;
            cp->pos = a;
            cp->newPos = a;
            cp->toolElemIndex = ca.elem.second.getIndex();
            cp->points.resize(6);
            cp->points[0] = a_last;
            cp->points[1] = a_p12_last[0];
            cp->points[2] = a_p12_last[1];
            cp->points[3] = a_i123_last[0];
            cp->points[4] = a_i123_last[1];
            cp->points[5] = a_i123_last[2];
            cp->init();
            //cp->pos = (*cp->mstate->getX())[cp->points[0]];
            cuttingPoints.push_back(cp);
        }
    }
    else
    {
        helper::vector<bool> contactUsed(ncontacts);
        for (unsigned int i=0;i<cuttingPoints.size();i++)
        {
            CuttingPoint* cp = cuttingPoints[i];
            if (cp->points.size()!=6)
            {
                std::cerr << "CuttingPoint "<<i<<" became INVALID."<<std::endl;
                cp->getContext()->removeObject(cp);
                delete cp;
                cuttingPoints.erase(cuttingPoints.begin()+i);
                --i;
                continue;
            }
            // update the cut position from the current mesh
            {
                Coord p = (*cp->mstate->getX())[cp->points[0]];
                cp->lastPos += p-cp->pos;
                cp->newPos += p-cp->pos;
                cp->pos = p;
                cp->lastPos = p;
            }
            // find the closest contact point
            Real dist = f_maxDistance.getValue();
            int cId = -1;
            Line edgeA(modelTool, cp->toolElemIndex);
            for (unsigned int j=0; j < ncontacts; ++j)
            {
                if (contactUsed[j]) continue;
                const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>::value_type& c = (*contacts)[j];
                // only consider a contact with an edge close to the previous edge
                Line edgeB(modelTool, c.elem.second.getIndex());
                if (edgeB.i1() != edgeA.i1() && edgeB.i1() != edgeA.i2() && edgeB.i2() != edgeA.i1() && edgeB.i2() != edgeA.i2())
                    continue;
                Real d = (c.point[0]-cp->pos).norm();
                if (d < dist)
                {
                    dist = d;
                    cId = j;
                }
            }
            if (cId >= 0)
            {
                std::cout << "Associating CuttingPoint "<<i<<" with contact "<<cId<<std::endl;
                contactUsed[cId] = true;
                const core::componentmodel::collision::TDetectionOutputVector<TriangleModel,LineModel>::value_type& cb = (*contacts)[cId];
                Coord a = cp->pos;
                Coord b = cb.point[0];
                unsigned int ind_ta = 0;
                unsigned int ind_tb = cb.elem.first.getIndex();
                // check if we are in the cutting direction of this edge
                {
                    Coord dp = b-a;
                    Coord n = modelTool->getNormal(cb.elem.second.getIndex());
                    Real cosAngle = modelTool->getCosAngle(cb.elem.second.getIndex());
                    if (dot(dp,n) < cosAngle*dp.norm())
                    {
                        std::cout << "Non cutting side of the edge. Ignoring motion..." << std::endl;
                        continue;
                    }
                }
                cp->newPos = b;
                cp->toolElemIndex = cb.elem.second.getIndex();
                if ((a-b).norm() < f_minDistance.getValue())
                {
                    std::cout << "distance "<<dist<<" smaller than limit "<<f_minDistance.getValue()<<std::endl;
                    continue;
                }
                // check if we cross at least an edge
                // this is verified when the new point is not in a triangle adjacent with the current cutting point
                topology::Triangle t = tsp->getTriangleSetTopologyContainer()->getTriangle(ind_tb);
                if (t[0]==cp->points[0] || t[1]==cp->points[0] || t[2]==cp->points[0])
                {
                    std::cout << "Still in the same triangle " << ind_tb << std::endl;
                    continue;
                }
                // Check if we are not too close to an edge of the triangle
                sofa::helper::vector< double > baryCoefs = tsp->getTriangleSetGeometryAlgorithms()->computeTriangleBarycoefs( b, ind_tb );
                if (baryCoefs.size() == 3 && (baryCoefs[0]<edgeDistance || baryCoefs[1]<edgeDistance || baryCoefs[2]<edgeDistance))
                {
                    std::cout << "Too close to an edge" << std::endl;
                    continue;
                }
                std::cout << "Continue CuttingPoint "<<i<<"..."<<std::endl;
                
                unsigned int a_last = 0;
                sofa::helper::vector< unsigned int > a_p12_last;
                sofa::helper::vector< unsigned int > a_i123_last;
                
                unsigned int b_last = cp->points[0];
                sofa::helper::vector< unsigned int > b_p12_last(2);
                b_p12_last[0] = cp->points[1];
                b_p12_last[1] = cp->points[2];
                sofa::helper::vector< unsigned int > b_i123_last(3);
                b_i123_last[0] = cp->points[3];
                b_i123_last[1] = cp->points[4];
                b_i123_last[2] = cp->points[5];
                
                sofa::helper::vector< sofa::helper::vector<unsigned int> > new_points;
                sofa::helper::vector< sofa::helper::vector<unsigned int> > closest_vertices;
                bool is_first_cut = false;
                bool is_fully_cut = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongPointsList(is_first_cut, a, b, ind_ta, ind_tb, 
                                                                                                   a_last, a_p12_last, a_i123_last,
                                                                                                   b_last, b_p12_last, b_i123_last, new_points, closest_vertices);
                is_fully_cut = false;
                if (!is_fully_cut && b_p12_last.size()==2 && b_i123_last.size()==3)
                {
                    // keep the cutting point
                    cp->lastPos = a;
                    cp->pos = b;
                    cp->newPos = b;
                    cp->toolElemIndex = cb.elem.second.getIndex();
                    cp->points.resize(6);
                    cp->points[0] = b_last;
                    cp->points[1] = b_p12_last[0];
                    cp->points[2] = b_p12_last[1];
                    cp->points[3] = b_i123_last[0];
                    cp->points[4] = b_i123_last[1];
                    cp->points[5] = b_i123_last[2];
                    cp->pos = (*cp->mstate->getX())[cp->points[0]];
                }
                else
                {
                    std::cerr << "CuttingPoint "<<i<<" stopped."<<std::endl;
                    cp->getContext()->removeObject(cp);
                    delete cp;
                    cuttingPoints.erase(cuttingPoints.begin()+i);
                    --i;
                    continue;
                }
            }
        }
    }
    
    detectionNP->setInstance(NULL);
    //std::cout << "CuttingManager: cut done" << std::endl;
}

void CuttingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::tree::AnimateBeginEvent* ev = */ dynamic_cast<simulation::tree::AnimateBeginEvent*>(event))
    {
    }
    if (/* simulation::tree::AnimateEndEvent* ev = */ dynamic_cast<simulation::tree::AnimateEndEvent*>(event))
    {
        doCut();
    }
}

void CuttingPoint::draw()
{
    if (!mstate) return;
    glDisable (GL_LIGHTING);
    glColor4f (1,1,0,1);
    glLineWidth(3);
    glBegin (GL_LINES);
    helper::gl::glVertexT(lastPos);
    helper::gl::glVertexT(pos);
    glEnd();
    glColor4f (0.5f,0.5f,0,1);
    glLineWidth(2);
    glBegin (GL_LINES);
    helper::gl::glVertexT(pos);
    helper::gl::glVertexT(newPos);
    glEnd();
    glPointSize(5);
    glColor4f (1,1,0,1);
    glBegin (GL_POINTS);
    const VecCoord& x = *mstate->getX();
    for (topology::PointSubset::const_iterator it = points.begin();
         it != points.end();
         ++it)
    {
        helper::gl::glVertexT(x[*it]);
        glColor4f (0.5f,0.5f,0,1);
    }
    glEnd();
    glLineWidth(1);
    glPointSize(1);
}    

} // namespace collision

} // namespace component

} // namespace sofa
