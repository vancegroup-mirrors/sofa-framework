/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <sofa/component/collision/RayPickInteractor.h>
#include <sofa/component/collision/RayContact.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/component/collision/SphereTreeModel.h>
#include <sofa/component/collision/DistanceGridCollisionModel.h>
#include <sofa/component/mapping/BarycentricMapping.h>
#include <sofa/component/mapping/RigidMapping.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/simulation/tree/DeleteVisitor.h>
#if defined (__APPLE__)
#include <OpenGL/gL.h>
#else
#include <GL/gl.h>
#endif
#include <sofa/component/topology/TriangleSetTopology.h>
//#include <sofa/component/topology/EdgeSetTopology.h>
#ifdef SOFA_GPU_CUDA
//#include <sofa/gpu/cuda/CudaMechanicalObject.inl>
//#include <sofa/gpu/cuda/CudaSpringForceField.inl>
#endif

namespace sofa
{

namespace component
{

namespace collision
{

#ifdef SOFA_GPU_CUDA
typedef TSphereModel<sofa::gpu::cuda::CudaVec3Types> CudaSphereModel;
#endif

using namespace sofa::defaulttype;

RayPickInteractor::RayPickInteractor()
: mm(NULL)
#ifdef SOFA_GPU_CUDA
, mmcuda(NULL)
#endif
, state(DISABLED), button(0)
{
    //setObject(this);
    transform.identity();
}

RayPickInteractor::~RayPickInteractor()
{
    for (unsigned int i=0;i<nodes.size();i++)
    {
        nodes[i]->execute<simulation::tree::DeleteVisitor>();
        nodes[i]->getParent()->removeChild(nodes[i]);
        delete nodes[i];
    }
    if (mm!=NULL)
        delete mm;
    mm = NULL;
    forcefields.clear();
    nodes.clear();
    attachedPoints.clear();
#ifdef SOFA_GPU_CUDA
    if (mmcuda!=NULL)
        delete mmcuda;
    mmcuda = NULL;
    cudaAttachedPoints.clear();
#endif
}

void RayPickInteractor::init()
{
    this->RayModel::init();
}


bool RayPickInteractor::isActive()
{
    return state==ACTIVE || state==PRESENT || state == FIRST_INPUT || state == IS_CUT; // state FIRST_INPUT controls the first input point for incision in a triangular mesh
}

/// Computation of a new simulation step.
void RayPickInteractor::updatePosition(double /*dt*/)
{

    int oldState = state;
    if (state == DISABLED && getNbRay()>0)
    { // we need to disable our interactor
        resize(0);
    }
    else if (state != DISABLED && getNbRay()==0)
    { // we need to enable our interactor
        std::cout << "ADD RAY"<<std::endl;
        setNbRay(1);
        getRay(0).l() = 10000;
    }

    if ((state == ACTIVE || state == FIRST_INPUT || state == IS_CUT) && !contacts.empty()) // state FIRST_INPUT controls the first input point for incision in a triangular mesh
    { // we need to attach a body
        // We find the first contact for each ray
        mm = new component::MechanicalObject<Vec3Types>();
        mm->setContext(getContext());
        mm->init();
#ifdef SOFA_GPU_CUDA
        mmcuda = new component::MechanicalObject<CudaVec3Types>();
        mmcuda->setContext(getContext());
        mmcuda->init();
#endif
        for (int r = 0; r < getNbRay(); r++)
        {

            Ray ray = getRay(r);
            double dist = 0;
            core::componentmodel::collision::DetectionOutput* collision = findFirstCollision(ray,&dist);
            if (collision==NULL)
                continue;
            core::CollisionElementIterator elem1 = collision->elem.first;
            core::CollisionElementIterator elem2 = collision->elem.second;
            Vector3 p1 = collision->point[0];
            Vector3 p2 = collision->point[1];
            if (elem2.getCollisionModel() == this)
            { // swap elements
                elem2 = collision->elem.first;
                elem1 = collision->elem.second;
                p2 = collision->point[0];
                p1 = collision->point[1];
            }
            if (dynamic_cast<SphereModel*>(elem2.getCollisionModel())!= NULL)
            {
                Sphere sphere(elem2);
                int index = attachedPoints.size();
                attachedPoints.push_back(std::make_pair(r,dist));
                switch (button)
                {
                case 0:
                    {
                        ContactForceField1* ff = new ContactForceField1(mm,sphere.getCollisionModel());
                        //ff->addSpring(index, sphere.getIndex(), 500, 50, (p1-sphere.center()).norm());
                        ff->addSpring(index, sphere.getIndex(), 1000*sphere.getContactStiffness(), 10, (p1-sphere.center()));
                        elem2.getCollisionModel()->getContext()->addObject(ff);
                        forcefields.push_back(ff);
                        break;
                    }
/*                case 1:
                  {
                        topology::EdgeSetTopology< Vec3Types >* topology = dynamic_cast< topology::EdgeSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
                        core::objectmodel::BaseContext* context = elem2.getCollisionModel()->getContext();
                        simulation::tree::GNode * node = dynamic_cast<simulation::tree::GNode *>(context);
                        assert( node != NULL ); // otherwise, collision model has no behavior model corresponding...
                        while ( node->mechanicalMapping )
                        {
                            node = node->parent;
                        }
                        component::MechanicalObject<Vec3Types>* mechModel = dynamic_cast< component::MechanicalObject< Vec3Types >* >(node->getMechanicalState());
                        if (topology != NULL && mechModel != NULL)
                        {
                            sofa::helper::vector< unsigned int > edgeShell = topology->getEdgeSetTopologyContainer()->getEdgeShell( (unsigned int) sphere.getIndex() );
                            sofa::helper::vector< unsigned int > toBeCut;
                            for (unsigned int i = 0; i < edgeShell.size(); ++i)
                            {
                                topology::Edge edge = topology->getEdgeSetTopologyContainer()->getEdge( edgeShell[i] );
                                Vector3 point1 = ( *mechModel->getX() )[ edge.first  ];
                                Vector3 point2 = ( *mechModel->getX() )[ edge.second ];
                                Vector3 m = (point1 + point2) /2;

                                Vector3 p = ray.origin();
                                Vector3 n = ray.direction();

                                double distance = (m-p)*(m-p) - ( (n*(m-p)) * (n*(m-p)) ) / (n*n);

                                if ( distance <= 20.0 )
                                    toBeCut.push_back( edgeShell[i] );


                            }
                            topology->getEdgeSetTopologyAlgorithms()->removeEdges( toBeCut );
                        }
                        break;

                    }*/
                }
            }
            else if (dynamic_cast<SphereTreeModel*>(elem2.getCollisionModel())!= NULL)
            {
                SingleSphere sphere(elem2);
                int index = attachedPoints.size();
                attachedPoints.push_back(std::make_pair(r,dist));
                switch (button)
                {
                case 0:
                    {
                        ContactForceField1* ff = new ContactForceField1(mm,sphere.getCollisionModel());
                        //ff->addSpring(index, sphere.getIndex(), 500, 50, (p1-sphere.center()).norm());
                        ff->addSpring(index, sphere.getIndex(), 1000*sphere.getContactStiffness(), 10, (p1-sphere.center()));
                        elem2.getCollisionModel()->getContext()->addObject(ff);
                        forcefields.push_back(ff);
                        break;
                    }
/*                case 1:
                    {
                        topology::EdgeSetTopology< Vec3Types >* topology = dynamic_cast< topology::EdgeSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
                        core::objectmodel::BaseContext* context = elem2.getCollisionModel()->getContext();
                        simulation::tree::GNode * node = dynamic_cast<simulation::tree::GNode *>(context);
                        assert( node != NULL ); // otherwise, collision model has no behavior model corresponding...
                        while ( node->mechanicalMapping )
                        {
                            node = node->parent;
                        }
                        component::MechanicalObject<Vec3Types>* mechModel = dynamic_cast< component::MechanicalObject< Vec3Types >* >(node->getMechanicalState());
                        if (topology != NULL && mechModel != NULL)
                        {
                            sofa::helper::vector< unsigned int > edgeShell = topology->getEdgeSetTopologyContainer()->getEdgeShell( (unsigned int) sphere.getIndex() );
                            sofa::helper::vector< unsigned int > toBeCut;
                            for (unsigned int i = 0; i < edgeShell.size(); ++i)
                            {
                                topology::Edge edge = topology->getEdgeSetTopologyContainer()->getEdge( edgeShell[i] );
                                Vector3 point1 = ( *mechModel->getX() )[ edge.first  ];
                                Vector3 point2 = ( *mechModel->getX() )[ edge.second ];
                                Vector3 m = (point1 + point2) /2;

                                Vector3 p = ray.origin();
                                Vector3 n = ray.direction();

                                double distance = (m-p)*(m-p) - ( (n*(m-p)) * (n*(m-p)) ) / (n*n);

                                if ( distance <= 20.0 )
                                    toBeCut.push_back( edgeShell[i] );


                            }
                            topology->getEdgeSetTopologyAlgorithms()->removeEdges( toBeCut );
                        }
                        break;

                    }*/
                }
            }
            else if ((dynamic_cast<TriangleModel*>(elem2.getCollisionModel()))!= NULL)
            {
                Triangle triangle(elem2);
                TriangleModel* model2 = triangle.getCollisionModel();
				if (button==1) {
					//std::cout << "Removing Triangle index= "<< elem2.getIndex()<< std::endl;
                    topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
					
					if (tsp){  
						sofa::helper::vector< unsigned int > triangles;
						triangles.push_back(elem2.getIndex());

						tsp->getTriangleSetTopologyAlgorithms()->removeTriangles(triangles);
					}
					continue;
				} 

				// Treatment of INCISION

				if (button==2) {

					//std::cout << "Input on triangle index= "<< elem2.getIndex() << ", coords = " << p2 << std::endl;
					topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
						                    
					if(state == FIRST_INPUT){

						if (tsp){  
							a_init[0] = p2[0]; 
							a_init[1] = p2[1];
							a_init[2] = p2[2];
							ind_ta_init = elem2.getIndex();

							is_first_cut = true;
						}

					}

					if(state == IS_CUT){
						
						if (tsp){  
			
							bool is_fully_cut = false;

							b_init[0] = p2[0];
							b_init[1] = p2[1];
							b_init[2] = p2[2];
							ind_tb_init = elem2.getIndex();

							const Vec<3,double>& a= (const Vec<3,double>) a_init;
							const Vec<3,double>& b= (const Vec<3,double>) b_init;

							const unsigned int &ind_ta = (const unsigned int) ind_ta_init;
							const unsigned int &ind_tb = (const unsigned int) ind_tb_init;
							
// 							unsigned int &new_ind_ta=(unsigned int) ind_ta;
// 							unsigned int &new_ind_tb=(unsigned int) ind_tb;

							//tsp->getTriangleSetTopologyAlgorithms()->RemoveAlongTrianglesList(a, b, ind_ta, ind_tb);
							//tsp->getTriangleSetTopologyAlgorithms()->Prepare_InciseAlongPointsList(a, b, ind_ta, ind_tb, new_ind_ta, new_ind_tb);

							unsigned int& b_last = b_last_init;
							sofa::helper::vector< unsigned int > &p12_last = p12_last_init;
							sofa::helper::vector< unsigned int >& i123_last = i123_last_init;

							bool is_prepared=!((a[0]==b[0] && a[1]==b[1] && a[2]==b[2]) || (ind_ta_init == ind_tb_init));

							if(is_prepared){

								sofa::helper::vector< sofa::helper::vector<unsigned int> > new_points_init;
								sofa::helper::vector< sofa::helper::vector<unsigned int> > closest_vertices_init;
								sofa::helper::vector< sofa::helper::vector<unsigned int> > &new_points = new_points_init;
								sofa::helper::vector< sofa::helper::vector<unsigned int> > &closest_vertices = closest_vertices_init;

								//std::cout << "is_first_cut = " << is_first_cut << std::endl;
								is_fully_cut = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongPointsList(is_first_cut, a, b, ind_ta, ind_tb, b_last, p12_last, i123_last, new_points, closest_vertices);
								//std::cout << "is_fully_cut = " << is_fully_cut << std::endl;
								is_first_cut=false;

							}else{
								is_fully_cut=false;
							}

							if(is_fully_cut){

								a_init[0] = p2[0]; 
								a_init[1] = p2[1];
								a_init[2] = p2[2];
								ind_ta_init = elem2.getIndex();

							}else{
								state = ATTACHED;
							}
						}
					}

					continue;
				} 

                simulation::tree::GNode* parent2 = dynamic_cast<simulation::tree::GNode*>(model2->getContext());
                if (parent2==NULL)
                {
                    std::cerr << "ERROR: RayPickInteractor triangle contacts only work for scenegraph scenes.\n";
                }
                else
                {
					component::MechanicalObject<Vec3Types>* mstate2 = NULL;
					simulation::tree::GNode* child2 = NULL;
					if(model2->getTopology()){ // Test added to prevent segmentation fault
						child2 = new simulation::tree::GNode("contactMouse");
						parent2->addChild(child2);
						child2->updateContext();
						mstate2 = new component::MechanicalObject<Vec3Types>;
						child2->addObject(mstate2);
						typedef mapping::BarycentricMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<TriangleModel::DataTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > TriangleMapping;
						typedef mapping::TopologyBarycentricMapper<topology::MeshTopology,TriangleModel::DataTypes, Vec3Types> TriangleMapper;
						TriangleMapper* mapper2 = new TriangleMapper(model2->getTopology());
						TriangleMapping* mapping2 = new TriangleMapping(model2->getMechanicalState(),mstate2,mapper2);
						child2->addObject(mapping2);
						mstate2->resize(1);
						(*mstate2->getX())[0] = p2;
						mapper2->clear();
						{
							int index2 = triangle.getIndex();
							if (index2 < model2->getTopology()->getNbTriangles())
							{
								mapper2->createPointInTriangle(p2, index2, model2->getMechanicalState()->getX());
							}
							else
							{
								mapper2->createPointInQuad(p2, (index2 - model2->getTopology()->getNbTriangles())/2, model2->getMechanicalState()->getX());
							}
						}
					}
					else
					{
					 	topology::TriangleSetTopology<TriangleModel::DataTypes>* t = dynamic_cast<topology::TriangleSetTopology<TriangleModel::DataTypes> *>(model2->getContext()->getMainTopology());
						if (t != NULL)
						{
							//const sofa::helper::vector<sofa::component::topology::Triangle> &ta=t->getTriangleSetTopologyContainer()->getTriangleArray();
							child2 = new simulation::tree::GNode("contactMouse");
							parent2->addChild(child2);
							child2->updateContext();
							mstate2 = new component::MechanicalObject<Vec3Types>;
							child2->addObject(mstate2);
							typedef mapping::BarycentricMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<TriangleModel::DataTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > TriangleMapping;
							typedef mapping::TopologyBarycentricMapper<topology::TriangleSetTopology<TriangleModel::DataTypes>, TriangleModel::DataTypes, Vec3Types> TriangleMapper;
							TriangleMapper* mapper2 = new TriangleMapper(t);
							TriangleMapping* mapping2 = new TriangleMapping(model2->getMechanicalState(),mstate2,mapper2);
							child2->addObject(mapping2);
							mstate2->resize(1);
							(*mstate2->getX())[0] = p2;
							mapper2->clear();
							int index2 = triangle.getIndex();
							mapper2->createPointInTriangle(p2, index2, model2->getMechanicalState()->getX());
						}
					}
					if (mstate2)
					{
						int index = attachedPoints.size();
						//double cdist = (p2-p1).norm();
						//if (cdist<1)
						//{
						//    dist -= 1;
						//    p1 -= ray.direction();
						//}
						attachedPoints.push_back(std::make_pair(r,dist));
						switch (button)
						{
						case 0:
							{
								ContactForceField1* ff = new ContactForceField1(mm,mstate2);
								//ff->addSpring(index, 0, 500, 50, (p1-p2).norm());
								ff->addSpring(index, 0, 1000*triangle.getContactStiffness(), 10, (p1-p2));
								child2->addObject(ff);
								forcefields.push_back(ff);
								nodes.push_back(child2); 
								break;
							}
							/*
						case 1:
							{
								ContactForceField2* ff = new ContactForceField2(mm,mstate2);
								ff->addConstraint(index, 0);
								child2->addObject(ff);
								forcefields.push_back(ff);
								nodes.push_back(child2);
								break;
							}*/
						}
					}else{
						continue; // to prevent segmentation fault
					}
                }
            }
            else if ((dynamic_cast<RigidDistanceGridCollisionModel*>(elem2.getCollisionModel()))!= NULL)
            {
                RigidDistanceGridCollisionElement distgrid(elem2);
                RigidDistanceGridCollisionModel* model2 = distgrid.getCollisionModel();
                simulation::tree::GNode* parent2 = dynamic_cast<simulation::tree::GNode*>(model2->getRigidModel()->getContext());
                if (parent2==NULL)
                {
                    std::cerr << "ERROR: RayPickInteractor triangle contacts only work for scenegraph scenes.\n";
                }
                else
                {
                    simulation::tree::GNode* child2 = new simulation::tree::GNode("contactMouse");
                    parent2->addChild(child2);
                    child2->updateContext();
                    component::MechanicalObject<Vec3Types>* mstate2 = new component::MechanicalObject<Vec3Types>;
                    child2->addObject(mstate2);
                    typedef mapping::RigidMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<RigidTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > DGMapping;
                    DGMapping* mapping2 = new DGMapping(model2->getRigidModel(),mstate2);
                    child2->addObject(mapping2);
                    mstate2->resize(1);
                    (*mstate2->getX())[0] = p2;
                    mapping2->init();
                    mapping2->updateMapping();
                    p2 = (*mstate2->getX())[0];
                    int index = attachedPoints.size();
                    //double cdist = (p2-p1).norm();
                    //if (cdist<1)
                    //{
                    //    dist -= 1;
                    //    p1 -= ray.direction();
                    //}
                    attachedPoints.push_back(std::make_pair(r,dist));
                    switch (button)
                    {
                    case 0:
                        {
                            ContactForceField1* ff = new ContactForceField1(mm,mstate2);
                            //ff->addSpring(index, 0, 500, 50, (p1-p2).norm());
                            ff->addSpring(index, 0, 1000*distgrid.getContactStiffness(), 10, (p1-p2));
                            child2->addObject(ff);
                            forcefields.push_back(ff);
                            nodes.push_back(child2); 
                            break;
                        }
/*                    case 1:
                        {
                            ContactForceField2* ff = new ContactForceField2(mm,mstate2);
                            ff->addConstraint(index, 0);
                            child2->addObject(ff);
                            forcefields.push_back(ff);
                            nodes.push_back(child2);
                            break;
                        }*/
                    }
                }
            }
            else if ((dynamic_cast<FFDDistanceGridCollisionModel*>(elem2.getCollisionModel()))!= NULL)
            {
                FFDDistanceGridCollisionElement distgrid(elem2);
                FFDDistanceGridCollisionModel* model2 = distgrid.getCollisionModel();
                simulation::tree::GNode* parent2 = dynamic_cast<simulation::tree::GNode*>(model2->getDeformModel()->getContext());
                if (parent2==NULL)
                {
                    std::cerr << "ERROR: RayPickInteractor FFD contacts only work for scenegraph scenes.\n";
                }
                else
                {
                    simulation::tree::GNode* child2 = new simulation::tree::GNode("contactMouse");
                    parent2->addChild(child2);
                    child2->updateContext();
                    component::MechanicalObject<Vec3Types>* mstate2 = new component::MechanicalObject<Vec3Types>;
                    child2->addObject(mstate2);
                    typedef mapping::BarycentricMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<FFDDistanceGridCollisionModel::DataTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > FFDMapping;
                    typedef mapping::TopologyBarycentricMapper<topology::RegularGridTopology,FFDDistanceGridCollisionModel::DataTypes, Vec3Types> FFDMapper;
                    FFDMapper* mapper2 = new FFDMapper(model2->getDeformGrid());
                    FFDMapping* mapping2 = new FFDMapping(model2->getDeformModel(),mstate2,mapper2);
                    child2->addObject(mapping2);
                    mstate2->resize(1);
                    (*mstate2->getX())[0] = p2;
                    mapper2->clear();
                    {
                        Vector3 bary;
                        int elem = model2->getDeformCube(distgrid.getIndex()).elem; //getDeformGrid()->findCube(p2,bary[0],bary[1],bary[2]);
                        bary = model2->getDeformCube(distgrid.getIndex()).baryCoords(p2);
                        mapper2->addPointInCube(elem,bary.ptr());
                    }
                    //mapping2->init();
                    mapping2->updateMapping();
                    p2 = (*mstate2->getX())[0];
                    int index = attachedPoints.size();
                    attachedPoints.push_back(std::make_pair(r,dist));
                    switch (button)
                    {
                    case 0:
                        {
                            ContactForceField1* ff = new ContactForceField1(mm,mstate2);
                            //ff->addSpring(index, 0, 500, 50, (p1-p2).norm());
                            ff->addSpring(index, 0, 1000*distgrid.getContactStiffness(), 10, (p1-p2));
                            child2->addObject(ff);
                            forcefields.push_back(ff);
                            nodes.push_back(child2); 
                            break;
                        }
/*                    case 1:
                        {
                            ContactForceField2* ff = new ContactForceField2(mm,mstate2);
                            ff->addConstraint(index, 0);
                            child2->addObject(ff);
                            forcefields.push_back(ff);
                            nodes.push_back(child2);
                            break;
                        }*/
                    }
                }
            }
#ifdef SOFA_GPU_CUDA
            else if (dynamic_cast<CudaSphereModel*>(elem2.getCollisionModel())!= NULL)
            {
                CudaSphereModel::Element sphere(elem2);
                int index = cudaAttachedPoints.size();
                cudaAttachedPoints.push_back(std::make_pair(r,dist));
                switch (button)
                {
                case 0:
                    {
                        CudaContactForceField1* ff = new CudaContactForceField1(mmcuda,sphere.getCollisionModel());
                        ff->addSpring(index, sphere.getIndex(), 1000*sphere.getContactStiffness(), 10, (p1-sphere.center()).norm());
                        elem2.getCollisionModel()->getContext()->addObject(ff);
                        ff->init();
                        forcefields.push_back(ff);
                        break;
                    }
                }
            }
#endif
            //ff->init();
        }
        mm->resize(attachedPoints.size());
#ifdef SOFA_GPU_CUDA
        mmcuda->resize(cudaAttachedPoints.size());
#endif
		if(state != FIRST_INPUT && state != IS_CUT){ // state == FIRST_INPUT is a particular case
			state = ATTACHED;
		}else{
			state = SECOND_INPUT; // force the state SECOND_INPUT
		}

    }
    else if (state != ATTACHED && mm!=NULL)
    { // we need to release the attached body
        for (unsigned int i=0;i<forcefields.size();i++)
        {
            forcefields[i]->getContext()->removeObject(forcefields[i]);
            delete forcefields[i];
        }
        for (unsigned int i=0;i<nodes.size();i++)
        {
            nodes[i]->execute<simulation::tree::DeleteVisitor>();
            nodes[i]->getParent()->removeChild(nodes[i]);
            delete nodes[i];
        }
        forcefields.clear();
        nodes.clear();
        if (mm!=NULL)
            delete mm;
        mm = NULL;
        attachedPoints.clear();
#ifdef SOFA_GPU_CUDA
        if (mmcuda!=NULL)
            delete mmcuda;
        mmcuda = NULL;
        cudaAttachedPoints.clear();
#endif
    }

    if (getNbRay()>=1)
    { // update current ray position
        Ray ray = getRay(0);
        ray.origin() = transform*Vec4d(0,0,0,1);
        ray.direction() = transform*Vec4d(0,0,1,0);
        ray.direction().normalize();
    }

    if (mm!=NULL)
    {
        Vec3Types::VecCoord& x = *mm->getX();
        for (unsigned int i=0;i<attachedPoints.size();i++)
        {
            Ray ray = getRay(attachedPoints[i].first);
            x[i] = ray.origin()+ray.direction()*attachedPoints[i].second;
        }
    }

#ifdef SOFA_GPU_CUDA
    if (mmcuda!=NULL)
    {
        CudaVec3Types::VecCoord& x = *mmcuda->getX();
        for (unsigned int i=0;i<cudaAttachedPoints.size();i++)
        {
            Ray ray = getRay(cudaAttachedPoints[i].first);
            x[i] = ray.origin()+ray.direction()*cudaAttachedPoints[i].second;
        }
    }
#endif

    if (oldState != state)
    {
        std::cout << "Interactor State: ";
        switch (state)
        {
        case DISABLED:
            std::cout << "DISABLED";
            break;
        case PRESENT:
            std::cout << "PRESENT";
            break;
        case ACTIVE:
            std::cout << "ACTIVE";
            break;
        case ATTACHED:
            std::cout << "ATTACHED";
            break;
		case FIRST_INPUT:
            std::cout << "FIRST_INPUT"; // first input point for incision in a triangular mesh
            break;
        case SECOND_INPUT:
            std::cout << "SECOND_INPUT"; // second input point for incision in a triangular mesh
            break;
		case IS_CUT:
            std::cout << "IS_CUT"; 
            break;
        }
        std::cout << std::endl;
    }
}

/// Interactor interface
void RayPickInteractor::newPosition(const Vector3& /*translation*/, const Quat& /*rotation*/, const Mat4x4d& transformation)
{
    transform = transformation;
}

void RayPickInteractor::newEvent(const std::string& command)
{
    int oldState = state;
    if (command == "show")
    {
        if (state==DISABLED)
            state = PRESENT;
    }
    else if (command == "pick" || command == "pick1") // detection of Left-click
    {
        if (state==DISABLED || state==PRESENT)
        {
            state = ACTIVE;
            button = 0;
        }
    }
    else if (command == "pick" || command == "pick2") // detection of Right-click
    {
        if (state==DISABLED || state==PRESENT)
        {
            state = ACTIVE;
            button = 1;
        }
    }
    else if (command == "pick3") // detection of Mid-click // command == "pick" || 
    {
        button = 2;

		if (state==SECOND_INPUT){
            state = IS_CUT;
		}else{
			state = FIRST_INPUT;
		}

    }
    else if (command == "release")
    {
		if (state==ACTIVE || state==ATTACHED){
            state = PRESENT;
		}
    }
    else if (command == "hide")
    {
        state = DISABLED;
    }
    if (oldState != state)
    {
        std::cout << "Interactor State: ";
        switch (state)
        {
        case DISABLED:
            std::cout << "DISABLED";
            break;
        case PRESENT:
            std::cout << "PRESENT";
            break;
        case ACTIVE:
            std::cout << "ACTIVE, button = "<<button;
            break;
        case ATTACHED:
            std::cout << "ATTACHED";
            break;
		case FIRST_INPUT:
            std::cout << "FIRST_INPUT, button = "<<button; // First input point for incision
            break;
        case SECOND_INPUT:
            std::cout << "SECOND_INPUT, button = "<<button; // Second input point for incision
            break;
		case IS_CUT:
            std::cout << "IS_CUT";
            break;
        }
        std::cout << std::endl;
    }
}

core::componentmodel::collision::DetectionOutput* RayPickInteractor::findFirstCollision(const Ray& ray, double* dist)
{
    core::componentmodel::collision::DetectionOutput* result = NULL;
    const Vector3& origin = ray.origin();
    const Vector3& direction = ray.direction();
    double l = ray.l();
    double mindist = 0;
    for (std::set
                <BaseRayContact*>::iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            const sofa::helper::vector<core::componentmodel::collision::DetectionOutput*>& collisions = (*it)->getDetectionOutputs();
            for (unsigned int i=0;i<collisions.size();i++)
            {
                core::componentmodel::collision::DetectionOutput* collision = collisions[i];
                if (collision->elem.second == ray)
                {
                    if (collision->elem.first.getCollisionModel()->isStatic())
                        continue;
                    double d = (collision->point[1] - origin)*direction;
                    if (d<0.0 || d>l)
                        continue;
                    if (result==NULL || d<mindist)
                    {
                        result = collision;
                        mindist = d;
                    }
                }
                else
                    if (collision->elem.first == ray)
                    {
                        if (collision->elem.second.getCollisionModel()->isStatic())
                            continue;
                        double d = (collision->point[0] - origin)*direction;
                        if (d<0.0 || d>l)
                            continue;
                        if (result==NULL || d<mindist)
                        {
                            result = collision;
                            mindist = d;
                        }
                    }
            }
        }
    if (dist!=NULL && result!=NULL)
        *dist = mindist;
    return result;
}

void RayPickInteractor::draw()
{
    if (getNbRay()>=1)
        this->RayModel::draw();
    if (state==PRESENT)
        glColor4f(0,1,0,1);
    //else if (state==ATTACHED) // commented : not to make visible the triangle stored in last position
    //    glColor4f(1,1,1,1);   // commented : not to make visible the triangle stored in last position
    else
        return;
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(3);
    for (int r = 0; r < getNbRay(); r++)
    {
        Ray ray = getRay(r);
        double dist = 0;
        core::componentmodel::collision::DetectionOutput* collision = findFirstCollision(ray,&dist);
        if (collision==NULL)
            continue;
        core::CollisionElementIterator elem = collision->elem.first;
        elem.draw();
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4f(1,1,1,1);
    for (unsigned int i=0;i<forcefields.size();i++)
    {
        if (dynamic_cast<core::VisualModel*>(forcefields[i])!=NULL)
        {
            // Hack to make forcefields visible
            bool b = forcefields[i]->getContext()->getShowInteractionForceFields();
            forcefields[i]->getContext()->setShowInteractionForceFields(true);
            dynamic_cast<core::VisualModel*>(forcefields[i])->draw();
            forcefields[i]->getContext()->setShowInteractionForceFields(b);
        }
    }
    glLineWidth(1);
}

} // namespace collision

} // namespace component

} // namespace sofa


