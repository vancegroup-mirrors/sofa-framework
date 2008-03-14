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
#include <sofa/component/mapping/SkinningMapping.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/simulation/tree/DeleteVisitor.h>
#include <sofa/component/topology/TriangleSetTopology.h>
#include <sofa/component/topology/TetrahedronSetTopology.h>
#include <sofa/component/topology/QuadSetTopology.h>
#include <sofa/component/topology/HexahedronSetTopology.h>
//#include <sofa/component/topology/EdgeSetTopology.h>
#ifdef SOFA_GPU_CUDA
//#include <sofa/gpu/cuda/CudaMechanicalObject.inl>
//#include <sofa/gpu/cuda/CudaSpringForceField.inl>
#endif
#include <sofa/helper/system/gl.h>

#include <sofa/core/componentmodel/topology/TopologicalMapping.h>

namespace sofa
{

namespace component
{

namespace collision
{

#ifdef SOFA_GPU_CUDA
typedef TSphereModel<sofa::gpu::cuda::CudaVec3Types> CudaSphereModel;
#endif

typedef mapping::SkinningMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<defaulttype::Rigid3Types>, core::componentmodel::behavior::MechanicalState<defaulttype::Vec3Types> > > MySkinningMapping;

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
				std::cout << "MODEL = SphereModel"<< std::endl;

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
				std::cout << "MODEL = SphereTreeModel"<< std::endl;

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
            else if((dynamic_cast<TriangleModel*>(elem2.getCollisionModel()))!= NULL)
            {
				std::cout << "MODEL = TriangleModel asked for input triangle = " << elem2.getIndex() << std::endl;

                Triangle triangle(elem2);
                TriangleModel* model2 = triangle.getCollisionModel();
				if (button==1) {
                                    TriangleSetModel* my_triangle_model = (dynamic_cast<TriangleSetModel*>(elem2.getCollisionModel()));
                                    if (my_triangle_model) {
					//std::cout << "Removing Triangle index= "<< elem2.getIndex()<< std::endl;

					sofa::helper::vector<unsigned int> my_Loc2GlobVec;
					bool is_TopologicalMapping = false;

					// Test if a TopologicalMapping exists :

					simulation::tree::GNode* parent2 = dynamic_cast<simulation::tree::GNode*>(model2->getContext());
					sofa::core::componentmodel::topology::TopologicalMapping *obj = dynamic_cast<sofa::core::componentmodel::topology::TopologicalMapping *>(*parent2->object.begin());

					for (simulation::tree::GNode::ObjectIterator it = parent2->object.begin(); it != parent2->object.end(); ++it)
					{
						//std::cout << "INFO : name of GNode = " << (*it)->getName() <<  std::endl;

						if (dynamic_cast<sofa::core::componentmodel::topology::TopologicalMapping *>(*it)!= NULL){

							obj = dynamic_cast<sofa::core::componentmodel::topology::TopologicalMapping *>(*it);
							is_TopologicalMapping=true;
							my_Loc2GlobVec = obj->getLoc2GlobVec();
							
						}
					}

					topology::TetrahedronSetTopology< Vec3Types >* tesp;

					if(is_TopologicalMapping){ 

						// try to catch the input topology of the TopologicalMapping (a TetrahedronSetTopology is expected)
						tesp = dynamic_cast<topology::TetrahedronSetTopology< Vec3Types >*>(obj->getFrom());
					}else{

						// try to catch the topology associated to the detected object (a TetrahedronSetTopology is first expected)
						tesp = dynamic_cast< topology::TetrahedronSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
					}

					if (tesp){
						
						sofa::core::componentmodel::topology::TopologyContainer *container=tesp->getTopologyContainer();
						sofa::component::topology::TetrahedronSetTopologyContainer *testc= dynamic_cast<sofa::component::topology::TetrahedronSetTopologyContainer *>(container);

						if (testc) {														

							sofa::helper::vector< unsigned int > tetrahedra;

							/* /// TEST to remove randomly a big number of tetrahedra

							unsigned int last_tetra;
							unsigned int nb_tetras = 0;

							
							while(nb_tetras < 150 && testc->getTetrahedronArray().size() != 0){ 

								last_tetra = rand()%(testc->getTetrahedronArray().size()); 
								tetrahedra.clear();
								tetrahedra.push_back(last_tetra);
								
								tesp->getTetrahedronSetTopologyAlgorithms()->removeTetrahedra(tetrahedra);
																
								nb_tetras+=1;
								
							}
							*////												

							unsigned int my_triangle_index;

							if(is_TopologicalMapping){								
								my_triangle_index = my_Loc2GlobVec[elem2.getIndex()];
							}else{
								my_triangle_index = (my_triangle_model->getLoc2GlobVec())[elem2.getIndex()];
							}

							std::cout << "TOPOLOGY = TetrahedronSetTopology, input triangle has global index = " << my_triangle_index <<  std::endl;

							if(my_triangle_index<(testc->getTetrahedronTriangleShellArray()).size()){

								sofa::helper::vector< unsigned int > tetras_to_remove = testc->getTetrahedronTriangleShell(my_triangle_index);

								if(tetras_to_remove.size()==1){
									
									unsigned int tetra_first=tetras_to_remove[0];									
									tetrahedra.push_back(tetra_first);
									
									unsigned int tetra_next;

									sofa::component::topology::TetrahedronTriangles adjacent_triangles = testc->getTetrahedronTriangles(tetra_first);
									for (unsigned int ti=0; ti<adjacent_triangles.size(); ti++){

										tetras_to_remove = testc->getTetrahedronTriangleShell(adjacent_triangles[ti]);
										
										if(tetras_to_remove.size()==2){

											if(tetras_to_remove[0]==tetra_first){
												tetra_next=tetras_to_remove[1];
											}else{
												tetra_next=tetras_to_remove[0];
											}

											tetrahedra.push_back(tetra_next);											
										}

									}
									
									tesp->getTetrahedronSetTopologyAlgorithms()->removeTetrahedra(tetrahedra);
									
							////
									// notify the end for the current sequence of topological change events
									tesp->getTetrahedronSetTopologyAlgorithms()->notifyEndingEvent();

									tesp->propagateTopologicalChanges();
							////
							
								}
							}else{
								//std::cout << "TOPOLOGY - WARN : size = " << (testc->getTetrahedronTriangleShellArray()).size() <<  std::endl;
							}
						}
					}else{

						// try to catch the topology associated to the detected object (a TriangleSetTopology is now expected)
						topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
						
						if (tsp){							

							std::cout << "TOPOLOGY = TriangleSetTopology"<< std::endl;
							std::cout << "TOPOLOGY = TriangleSetTopology, input triangle = " << elem2.getIndex() <<  std::endl;

							sofa::helper::vector< unsigned int > triangles;

							unsigned int my_triangle_index = elem2.getIndex();

							triangles.push_back(my_triangle_index);
							tsp->getTriangleSetTopologyAlgorithms()->removeTriangles(triangles, true, true);

							/* /// TEST to remove randomly a big number of triangles

							unsigned int last_triangle;
							unsigned int nb_triangles = 0;

							sofa::core::componentmodel::topology::TopologyContainer *container=tsp->getTopologyContainer();
							sofa::component::topology::TriangleSetTopologyContainer *tstc= dynamic_cast<sofa::component::topology::TriangleSetTopologyContainer *>(container);
							
							while(nb_triangles < 50 && tstc->getTriangleArray().size() != 0){

								last_triangle = rand()%(tstc->getTriangleArray().size());
								triangles.clear();
								triangles.push_back(last_triangle);
								
								tsp->getTriangleSetTopologyAlgorithms()->removeTriangles(triangles);
								
								nb_triangles+=1;
								
							}

							*/

							// notify the end for the current sequence of topological change events
							tsp->getTriangleSetTopologyAlgorithms()->notifyEndingEvent();

							tsp->propagateTopologicalChanges();
						}
					}

					/*
					topology::HexahedronSetTopology< Vec3Types >* hsp = dynamic_cast< topology::HexahedronSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
					if (hsp){

						std::cout << "TOPOLOGY = HexahedronSetTopology"<< std::endl;

						sofa::helper::vector< unsigned int > hexahedra;
						hexahedra.push_back(elem2.getIndex());

						hsp->getHexahedronSetTopologyAlgorithms()->removeHexahedra(hexahedra);

					}else{
						topology::QuadSetTopology< Vec3Types >* qsp = dynamic_cast< topology::QuadSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
						if (qsp){

							std::cout << "TOPOLOGY = QuadSetTopology"<< std::endl;

							sofa::helper::vector< unsigned int > quads;
							quads.push_back(elem2.getIndex());

							qsp->getQuadSetTopologyAlgorithms()->removeQuads(quads);
						}

					}
					*/
                                    }

					continue;
				} 

				// Treatment of INCISION

				if (button==2) {

					// Test if a TopologicalMapping (by default from TetrahedronSetTopology to TriangleSetTopology) exists :

					bool is_TopologicalMapping = false;

					simulation::tree::GNode* parent2 = dynamic_cast<simulation::tree::GNode*>(model2->getContext());

					for (simulation::tree::GNode::ObjectIterator it = parent2->object.begin(); it != parent2->object.end(); ++it)
					{
						//std::cout << "INFO : name of GNode = " << (*it)->getName() <<  std::endl;

						if (dynamic_cast<sofa::core::componentmodel::topology::TopologicalMapping *>(*it)!= NULL){

							is_TopologicalMapping=true;							
						}
					}

					// try to catch the topology associated to the detected object (a TriangleSetTopology is expected)
					topology::TriangleSetTopology< Vec3Types >* tsp = dynamic_cast< topology::TriangleSetTopology< Vec3Types >* >( elem2.getCollisionModel()->getContext()->getMainTopology() );
						
					if(!is_TopologicalMapping){ // no TopologicalMapping
						
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

								unsigned int& a_last = a_last_init;
								sofa::helper::vector< unsigned int >& a_p12_last = a_p12_last_init;
								sofa::helper::vector< unsigned int >& a_i123_last = a_i123_last_init;

								unsigned int& b_last = b_last_init;
								sofa::helper::vector< unsigned int >& b_p12_last = b_p12_last_init;
								sofa::helper::vector< unsigned int >& b_i123_last = b_i123_last_init;

								bool is_prepared=!((a[0]==b[0] && a[1]==b[1] && a[2]==b[2]) || (ind_ta_init == ind_tb_init));

								if(is_prepared){

									sofa::helper::vector< sofa::helper::vector<unsigned int> > new_points_init;
									sofa::helper::vector< sofa::helper::vector<unsigned int> > closest_vertices_init;
									sofa::helper::vector< sofa::helper::vector<unsigned int> > &new_points = new_points_init;
									sofa::helper::vector< sofa::helper::vector<unsigned int> > &closest_vertices = closest_vertices_init;

									is_fully_cut = tsp->getTriangleSetTopologyAlgorithms()->InciseAlongPointsList(is_first_cut, a, b, ind_ta, ind_tb, a_last, a_p12_last, a_i123_last, b_last, b_p12_last, b_i123_last, new_points, closest_vertices);
									
									// notify the end for the current sequence of topological change events
									tsp->getTriangleSetTopologyAlgorithms()->notifyEndingEvent();

									tsp->propagateTopologicalChanges();

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
            if (parent2->mechanicalMapping != NULL && parent2->mechanicalMapping->isMechanical() == false)
            {
                core::componentmodel::behavior::BaseMechanicalMapping* mapping = parent2->mechanicalMapping;
                if (MySkinningMapping * m = dynamic_cast<MySkinningMapping*>(mapping))
                {
                    parent2 = dynamic_cast<simulation::tree::GNode*>(m->getMechFrom()->getContext());
                    child2 = new simulation::tree::GNode("contactMouse");
                    parent2->addChild(child2);
                    child2->updateContext();
                    mstate2 = new component::MechanicalObject<Vec3Types>;
                    child2->addObject(mstate2);
                    MySkinningMapping * mapping2 = new MySkinningMapping(m->getFromModel(), mstate2);
                    child2->addObject(mapping2);
                    mstate2->resize(1);
                    (*mstate2->getX())[0] = p2;
                    const sofa::helper::vector<double>& incoefs = m->getWeightCoefs();
                    const sofa::helper::vector<unsigned int>& inrefs = m->getRepartition();
                    int nbr = m->getNbRefs();
                    sofa::helper::vector<double> coefs;
                    sofa::helper::vector<unsigned int> refs;
                    int v[3] = { triangle.p1Index(), triangle.p2Index(), triangle.p3Index() };
                    double baryCoords[3];
                    const Vector3 p0 = triangle.p1();
                    const Vector3 pA = triangle.p2() - p0;
                    const Vector3 pB = triangle.p3() - p0;
                    Vector3 pos = p2 - p0;
                    // First project to plane
                    Vector3 normal = cross(pA, pB);
                    double norm2 = normal.norm2();
                    pos -= normal*((pos*normal)/norm2);
                    baryCoords[1] = sqrt(cross(pB, pos).norm2() / norm2);
                    baryCoords[2] = sqrt(cross(pA, pos).norm2() / norm2);
                    baryCoords[0] = 1.0-baryCoords[1]-baryCoords[2];
                    for (int i=0;i<3;i++)
                    {
                        int input = v[i]*nbr;
                        for (int c=0;c<nbr;c++)
                        {
                            coefs.push_back(incoefs[input+c]*baryCoords[i]);
                            refs.push_back(inrefs[input+c]);
                        }
                    }
                    mapping2->setComputeWeights(false);
                    mapping2->setNbRefs(refs.size());
                    mapping2->setRepartition(refs);
                    mapping2->setWeightCoefs(coefs);
                    mapping2->init();
                }
                else continue;
            }
            else
					if(TriangleMeshModel* tmodel2 = dynamic_cast<TriangleMeshModel*>(model2)){
						child2 = new simulation::tree::GNode("contactMouse");
						parent2->addChild(child2);
						child2->updateContext();
						mstate2 = new component::MechanicalObject<Vec3Types>;
						child2->addObject(mstate2);
						typedef mapping::BarycentricMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<TriangleModel::DataTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > TriangleMapping;
						typedef mapping::TopologyBarycentricMapper<TriangleMeshModel::Topology,TriangleMeshModel::DataTypes, Vec3Types> TriangleMapper;
						TriangleMapper* mapper2 = new TriangleMapper(tmodel2->getTopology());
						TriangleMapping* mapping2 = new TriangleMapping(tmodel2->getMechanicalState(),mstate2,mapper2);
						child2->addObject(mapping2);
						mstate2->resize(1);
						(*mstate2->getX())[0] = p2;
						mapper2->clear();
						{
							int index2 = triangle.getIndex();
							if (index2 < tmodel2->getTopology()->getNbTriangles())
							{
								mapper2->createPointInTriangle(p2, index2, tmodel2->getMechanicalState()->getX());
							}
							else
							{
								mapper2->createPointInQuad(p2, (index2 - tmodel2->getTopology()->getNbTriangles())/2, tmodel2->getMechanicalState()->getX());
							}
						}
					}
					else if(TriangleSetModel* tmodel2 = dynamic_cast<TriangleSetModel*>(model2)){
					 	TriangleSetModel::Topology* t = tmodel2->getTopology();
						if (t != NULL)
						{
							//const sofa::helper::vector<sofa::component::topology::Triangle> &ta=t->getTriangleSetTopologyContainer()->getTriangleArray();
							child2 = new simulation::tree::GNode("contactMouse");
							parent2->addChild(child2);
							child2->updateContext();
							mstate2 = new component::MechanicalObject<Vec3Types>;
							child2->addObject(mstate2);
							typedef mapping::BarycentricMapping<core::componentmodel::behavior::MechanicalMapping<core::componentmodel::behavior::MechanicalState<TriangleModel::DataTypes>, core::componentmodel::behavior::MechanicalState<Vec3Types> > > TriangleMapping;
							typedef mapping::TopologyBarycentricMapper<TriangleSetModel::Topology, TriangleSetModel::DataTypes, Vec3Types> TriangleMapper;
							TriangleMapper* mapper2 = new TriangleMapper(t);
							TriangleMapping* mapping2 = new TriangleMapping(model2->getMechanicalState(),mstate2,mapper2);
							child2->addObject(mapping2);
							mstate2->resize(1);
							(*mstate2->getX())[0] = p2;
							mapper2->clear();
							int index2 = triangle.getIndex();
							mapper2->createPointInTriangle(p2, tmodel2->convertLoc2Glob(index2), tmodel2->getMechanicalState()->getX());
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
				std::cout << "MODEL = RigidDistanceGridCollisionModel"<< std::endl;

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
				std::cout << "MODEL = FFDDistanceGridCollisionModel"<< std::endl;

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
                    if (!collision->elem.first.getCollisionModel()->isSimulated())
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
                        if (!collision->elem.second.getCollisionModel()->isSimulated())
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


