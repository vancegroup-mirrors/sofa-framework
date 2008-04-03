#include "FractureManager.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(FractureManager)

int FractureManagerClass = core::RegisterObject("Manager handling fracture operations between a SharpLineModel and a TriangleModel")
.add< FractureManager >()
;


FractureManager::FractureManager():
min_elongation( initData(&min_elongation, (Real)0.5f, "min_elongation", " "))
{
	this->f_listening.setValue(true);
	fracturePointDetected = false;
}

FractureManager::~FractureManager()
{
}

void FractureManager::init()
{
    if (f_modelSurface.getValue().empty())
    {
        // we look for a fraturable TriangleSetModel.
        // this currently means a model with a TriangleSetTopology but not a TetrahedronSetTopology.
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
            modelSurface = m; // we found a fracturable object
            break;
        }
    }
    else
        modelSurface = getContext()->get<TriangleSetModel>(f_modelSurface.getValue());
    bool error = false;
    if (modelSurface == NULL) { std::cerr << "FractureManager: modelSurface not found\n"; error = true;  return; }
}

void FractureManager::reset()
{
    fracturingPoints.clear();
}



int FractureManager::getFractureEdgeFromPoint(int pt, Real& alpha)
{
//		std::cout << "get fracture edge from point "<<pt<<std::endl;
		topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( modelSurface->getContext()->getMainTopology() );

		sofa::helper::vector< unsigned int > triangleVertexShell = tsp->getTriangleSetTopologyContainer()->getTriangleVertexShell(pt);
		sofa::helper::vector< unsigned int > edgeVertexShell = tsp->getTriangleSetTopologyContainer()->getEdgeVertexShell(pt);

		bool on_border = false;
		for (unsigned int i=0; i<edgeVertexShell.size(); i++)
		{
			sofa::helper::vector< unsigned int > triangleEdgeShell = tsp->getTriangleSetTopologyContainer()->getTriangleEdgeShell(edgeVertexShell[i]);
			if (triangleEdgeShell.size() == 1)
			{
				on_border = true;
				break;
			}
		}
		if (!on_border) return -1;

		/** computes the normal of the triangle vertex shell */
		Coord n;
		for (unsigned int i=0; i<triangleVertexShell.size(); i++)
		{
			n += tsp->getTriangleSetGeometryAlgorithms()->computeTriangleNormal(triangleVertexShell[i]);
		}

		Real edgeLength, restEdgeLength;
		sofa::helper::vector< Real >alphaVector;
		topology::Edge edge;
		sofa::helper::vector< Coord >u, d;
		sofa::helper::vector< Real > l,l0;
		sofa::helper::vector< bool > onBorder;
		Real alf=0.0;

		for (unsigned int i=0; i<edgeVertexShell.size(); i++)
		{
			edge = tsp->getEdgeSetTopologyContainer()->getEdge(edgeVertexShell[i]);
			edgeLength = tsp->getEdgeSetGeometryAlgorithms()->computeEdgeLength(edgeVertexShell[i]);
			restEdgeLength = tsp->getEdgeSetGeometryAlgorithms()->computeRestEdgeLength(edgeVertexShell[i]);
			l.push_back(edgeLength); /** edge length */
			l0.push_back(restEdgeLength); /** initial edge length */
			if ((int)edge.first == pt)
				u.push_back((*modelSurface->getMechanicalState()->getX())[edge.second] - (*modelSurface->getMechanicalState()->getX())[edge.first]);
			else
				u.push_back((*modelSurface->getMechanicalState()->getX())[edge.first] - (*modelSurface->getMechanicalState()->getX())[edge.second]);
			d.push_back(cross(n,u.back()));
			sofa::helper::vector< unsigned int > triangleEdgeShell = tsp->getTriangleSetTopologyContainer()->getTriangleEdgeShell(edgeVertexShell[i]);
			onBorder.push_back(triangleEdgeShell.size()==1);

			u.back().normalize();
			d.back().normalize();
		}

		/** alpha computations */

		double maxBorderFactor = 0;
		for (unsigned int j=0; j<edgeVertexShell.size(); j++)
		{
			alf = 0;
			double udAddition = 0.0;
			double borderFactor = 0;
			for (unsigned int i=0; i<edgeVertexShell.size(); i++)
			{
				if (l[i] > l0[i])
				{
					alf += fabs(((l[i]-l0[i])/l0[i])*(u[i]*d[j]));
					udAddition += (u[i]*d[j]);
				}
				if (onBorder[i])
				{
					double b = (1-u[i]*u[j])/2;
					if (b > borderFactor) borderFactor = b;
				}
			}
			//if (borderFactor > 0.5f) borderFactor = 1;
			if (borderFactor > maxBorderFactor)
				maxBorderFactor = borderFactor;
			if (udAddition != 0.0)
				alf /= udAddition;
			alf *= borderFactor;
			alphaVector.push_back(alf);
		}
		if (maxBorderFactor > 0)
		for (unsigned int j=0; j<edgeVertexShell.size(); j++)
		{
			alphaVector[j] /= maxBorderFactor;
		}
		alf = 0.0;
		int fracturedEdge = -1;
		for(unsigned int i=0; i<alphaVector.size(); i++)
		{
			/** select the bigger alpha for an edge between almost two triangles */
			sofa::helper::vector< unsigned int > triangleEdgeShell = tsp->getTriangleSetTopologyContainer()->getTriangleEdgeShell(edgeVertexShell[i]);
			if ((alphaVector[i]>alf) && (triangleEdgeShell.size() >= 2) && (alphaVector[i])>(min_elongation.getValue()))
			{
				alf = alphaVector[i];
				fracturedEdge = i;
			}
		}
		alpha = alf;
		if (fracturedEdge == -1) return -1;
		else
		{
//			std::cout << "-> edge "<<edgeVertexShell[fracturedEdge]<<std::endl;
			return edgeVertexShell[fracturedEdge];
		}
}

void FractureManager::doFracture()
{
    if (modelSurface==NULL) return;
    sofa::core::componentmodel::topology::BaseTopology* bt = dynamic_cast<sofa::core::componentmodel::topology::BaseTopology *>(modelSurface->getContext()->getMainTopology());
    if (bt == NULL) return;
    sofa::core::componentmodel::topology::TopologyContainer *container=bt->getTopologyContainer();
    sofa::component::topology::TriangleSetTopologyContainer *tstc= dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer *>(container);
    if (tstc == NULL) return;
	topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( modelSurface->getContext()->getMainTopology() );

	if (fracturingPoints.empty())
	{ // look on all points

		unsigned int nbp = modelSurface->getMechanicalState()->getX()->size();
		int max_pt = -1;
		Real max_alf = 0;
		int max_edge = -1;
		for (unsigned int pt=0;pt<nbp;++pt)
		{
			Real alf = 0;
			int fracturedEdge = -1;
			fracturedEdge = getFractureEdgeFromPoint(pt,alf);
			if (alf > max_alf && fracturedEdge != -1)
			{
				max_pt = pt;
				max_alf = alf;
				max_edge = fracturedEdge;
			}
		} // for pt

		int pt = max_pt;
		int fracturedEdge = max_edge;
		int indexFracturedPoint = -1;
		if (fracturedEdge != -1)
		{
		    if ((int)tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).first == pt)
			{
				indexFracturedPoint = tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).second;
			}
			else
			{
				indexFracturedPoint = tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).first;
			}

			int result = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongEdge(fracturedEdge);
			if (result == 1)
			{
				std::cout << " edge fractured " << std::endl;
				FracturingPoint* fp = new FracturingPoint;
				fp->points.push_back(indexFracturedPoint);
				modelSurface->getContext()->addObject(fp);
				fp->init();
				fp->pos = (*fp->mstate->getX())[fp->points[0]];
				fp->newPos = fp->pos;
				fracturingPoints.push_back(fp);
			}
		}
	}
	else
	{ // update existing fracturing points

		int indexFracturedPoint = -1;

		for (unsigned int i=0;i<fracturingPoints.size();i++)
		{

			FracturingPoint* fp = fracturingPoints[i];
			fp->pos = (*fp->mstate->getX())[fp->points[0]];
			fp->newPos = fp->pos;

			bool erase = false;

			int pt = fp->points[0];

			if (tstc->getTriangleVertexShell(pt) == tstc->getEdgeVertexShell(pt))
			{
				erase = true;
				std::cerr << "FracturingPoint "<<i<<" inside the surface."<<std::endl;
			}
			else
			{

				Real max_alf = 0;
				int fracturedEdge = -1;

				fracturedEdge = getFractureEdgeFromPoint(pt,max_alf);
				indexFracturedPoint = -1;
				if (fracturedEdge != -1)
				{
				    if ((int)tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).first == pt)
					{
						indexFracturedPoint = tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).second;
					}
					else
					{
						indexFracturedPoint = tsp->getEdgeSetTopologyContainer()->getEdge(fracturedEdge).first;
					}

					int result = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongEdge(fracturedEdge);
					if (result == 1)
					{
						fracturePointDetected = true;
						std::cout << " edge fractured " << std::endl;
						fp->points[0] = indexFracturedPoint;
						fp->pos = (*fp->mstate->getX())[fp->points[0]];
						fp->newPos = fp->pos;
					}
					else if (result == 2)
					{
						std::cerr << "FracturingPoint "<<i<<" finished."<<std::endl;
						erase = true;
					}
				}
			}
			if (erase)
			{
				fp->getContext()->removeObject(fp);
				delete fp;
				fracturingPoints.erase(fracturingPoints.begin()+i);
				--i;
				continue;
			}
		}
	}
}

void FractureManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<simulation::tree::AnimateBeginEvent*>(event))
    {
    }
    if (dynamic_cast<simulation::tree::AnimateEndEvent*>(event))
    {
        doFracture();
    }
}

void FracturingPoint::draw()
{
    if (!mstate) return;
    glDisable (GL_LIGHTING);
    //glColor4f (1,1,0,1);
    //glLineWidth(3);
    //glBegin (GL_LINES);
    //helper::gl::glVertexT(lastPos);
    //helper::gl::glVertexT(pos);
    //glEnd();
    glColor4f (1.0f,1.0f,1.0f,1);
    glPointSize(4);
	glBegin (GL_POINTS);
    helper::gl::glVertexT(pos);
    helper::gl::glVertexT(newPos);
    glEnd();
    //glPointSize(5);
    //glColor4f (1,1,0,1);
    //glBegin (GL_POINTS);
    //const VecCoord& x = *mstate->getX();
    //for (topology::PointSubset::const_iterator it = points.begin();
    //     it != points.end();
    //     ++it)
    //{
    //    helper::gl::glVertexT(x[*it]);
    //    glColor4f (0.5f,0.5f,0,1);
    //}
    //glEnd();
    //glLineWidth(1);
    //glPointSize(1);
}    

} // namespace collision

} // namespace component

} // namespace sofa
