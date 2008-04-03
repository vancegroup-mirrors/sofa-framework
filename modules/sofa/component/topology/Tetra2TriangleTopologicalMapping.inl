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
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRA2TRIANGLETOPOLOGICALMAPPING_INL
#define SOFA_COMPONENT_TOPOLOGY_TETRA2TRIANGLETOPOLOGICALMAPPING_INL

#include <sofa/component/topology/Tetra2TriangleTopologicalMapping.h>

#include <sofa/core/componentmodel/topology/Topology.h>

#include <sofa/component/topology/PointSetTopology.h>
#include <sofa/component/topology/TriangleSetTopology.h>
#include <sofa/component/topology/TetrahedronSetTopology.h>

#include <sofa/defaulttype/Vec.h>
#include <map>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa
{

namespace component
{

namespace topology
{

using namespace sofa::defaulttype;
using namespace sofa::core::componentmodel::behavior;

using namespace sofa::component::topology;
using namespace sofa::core::componentmodel::topology;

template <class In, class Out>
Tetra2TriangleTopologicalMapping<In,Out>::Tetra2TriangleTopologicalMapping(In* from, Out* to)
:
fromModel(from), toModel(to), 
object1(initData(&object1, std::string("../.."), "object1", "First object to map")),
object2(initData(&object2, std::string(".."), "object2", "Second object to map"))
{
}


template <class In, class Out>
Tetra2TriangleTopologicalMapping<In,Out>::~Tetra2TriangleTopologicalMapping()
{
}

template <class In, class Out>
In* Tetra2TriangleTopologicalMapping<In,Out>::getFromModel()
{
	return this->fromModel;
}

template <class In, class Out>
Out* Tetra2TriangleTopologicalMapping<In,Out>::getToModel()
{
	return this->toModel;
}

template <class In, class Out>
objectmodel::BaseObject* Tetra2TriangleTopologicalMapping<In,Out>::getFrom()
{
	return this->fromModel;
}

template <class In, class Out>
objectmodel::BaseObject* Tetra2TriangleTopologicalMapping<In,Out>::getTo()
{
	return this->toModel;
}


template <class In, class Out>
void Tetra2TriangleTopologicalMapping<In,Out>::setModels(In* from, Out* to){
	this->fromModel = from;
	this->toModel = to;
}


template <class In, class Out>
void Tetra2TriangleTopologicalMapping<In,Out>::init()
{
	//std::cout << "INFO_print : init Tetra2TriangleTopologicalMapping" << std::endl;

	// INITIALISATION of TRIANGULAR mesh from TETRAHEDRAL mesh :

	TopologyContainer *from_container=fromModel->getTopologyContainer();
	TetrahedronSetTopologyContainer *from_testc= dynamic_cast<TetrahedronSetTopologyContainer *>(from_container);
	
	if (from_testc) {
		
		//std::cout << "INFO_print : TopologicalMapping - from = tetra" << std::endl;

		TopologyContainer *to_container=toModel->getTopologyContainer();

		TriangleSetTopologyContainer *to_tstc= dynamic_cast<TriangleSetTopologyContainer *>(to_container);
		
		if (to_tstc) {

			//std::cout << "INFO_print : TopologicalMapping - to = triangle" << std::endl;

			TriangleSetTopologyModifier< Vec3Types >* to_tstm  = static_cast< TriangleSetTopologyModifier< Vec3Types >* >(toModel->getTopologyModifier());

			const sofa::helper::vector<Triangle> &triangleArray=from_testc->getTriangleArray();
			
			unsigned int nb_visible_triangles = 0;

			Loc2GlobVec.clear();
			Glob2LocMap.clear();

			for (unsigned int i=0; i<triangleArray.size(); ++i) {

					if (from_testc->getTetrahedronTriangleShell(i).size()==1) {							 

							to_tstm->addTriangle(triangleArray[i]);

							Loc2GlobVec.push_back(i);
							Glob2LocMap[i]=Loc2GlobVec.size()-1;

							nb_visible_triangles+=1;
					}
			}			

			TriangleSetTopology< Vec3Types >* to_tst = dynamic_cast< TriangleSetTopology< Vec3Types > *>( toModel );
			to_tst->getTriangleSetTopologyAlgorithms()->notifyEndingEvent();
		}
		
	}
}

template <class In, class Out>
void Tetra2TriangleTopologicalMapping<In,Out>::updateTopologicalMapping(){

	bool is_debug = false;

	// INITIALISATION of TRIANGULAR mesh from TETRAHEDRAL mesh :

	TopologyContainer *from_container=fromModel->getTopologyContainer();
	TetrahedronSetTopologyContainer *from_testc= dynamic_cast<TetrahedronSetTopologyContainer *>(from_container);
	
	if (from_testc) {

		TopologyContainer *to_container=toModel->getTopologyContainer();
		TriangleSetTopologyContainer *to_tstc= dynamic_cast<TriangleSetTopologyContainer *>(to_container);
		
		TriangleSetTopologyModifier< Vec3Types >* to_tstm  = static_cast< TriangleSetTopologyModifier< Vec3Types >* >(toModel->getTopologyModifier());

		if (to_tstc) {

			std::list<const TopologyChange *>::const_iterator itBegin=fromModel->firstChange();
			std::list<const TopologyChange *>::const_iterator itEnd=fromModel->lastChange();

			while( itBegin != itEnd )
			{
				TopologyChangeType changeType = (*itBegin)->getChangeType();
				// Since we are using identifier, we can safely use C type casts.

				TriangleSetTopology< Vec3Types >* to_tst = dynamic_cast< TriangleSetTopology< Vec3Types > *>( toModel );

				/////

				if(is_debug && changeType == core::componentmodel::topology::ENDING_EVENT){

					unsigned int my_size = 0;
					
					my_size = from_testc->getTetrahedronTriangleShellArray().size();
					
					//std::cout << "=============================================== triangles.size() = "<< triangles.size() << std::endl;
					//std::cout << "=============================================== Loc2GlobVec.size() = "<< Loc2GlobVec.size() << std::endl;
					//std::cout << "=============================================== Glob2LocMap.size() = "<< Glob2LocMap.size() << std::endl;

					
					// TEST 1
					for(unsigned int i_check= 0; i_check <Loc2GlobVec.size(); ++i_check){

						if(i_check!=Glob2LocMap[Loc2GlobVec[i_check]]){
							std::cout << "INFO_print : TopologicalMapping - Glob2LocMap fails at i_check = "<< i_check << std::endl;
						}

					}

					// TEST 2
					std::map<unsigned int, unsigned int>::iterator iter_check = Glob2LocMap.begin();
					while(iter_check != Glob2LocMap.end()){

						unsigned int my_glob = iter_check->first;
						//unsigned int my_loc = iter_check->second;
						iter_check++;

						if(my_glob!=Loc2GlobVec[Glob2LocMap[my_glob]]){
							std::cout << "INFO_print : TopologicalMapping - Loc2GlobVec fails at my_glob = "<< my_glob << std::endl;
						}

						if(my_glob>=my_size){
							std::cout << "INFO_print : TopologicalMapping - Glob2LocMap gives too big my_glob = "<< my_glob << std::endl;
						}
					}
					
					// TEST 3
					if(from_testc){

						for(unsigned int j_check= 0; j_check < my_size; ++j_check){

							if(from_testc->getTetrahedronTriangleShell(j_check).size()==1){

								std::map<unsigned int, unsigned int>::iterator iter_j = Glob2LocMap.find(j_check);
								if(iter_j == Glob2LocMap.end() ) {
									std::cout << "INFO_print : TopologicalMapping - Glob2LocMap should have the visible triangle j_check = "<< j_check << std::endl;
								}
								
							}else{

								std::map<unsigned int, unsigned int>::iterator iter_j = Glob2LocMap.find(j_check);
								if(iter_j != Glob2LocMap.end() ) {
									std::cout << "INFO_print : TopologicalMapping - Glob2LocMap should NOT have the INvisible triangle j_check = "<< j_check << std::endl;
								}
							}
						}
					}

					// TEST_END

				}

				/////

				switch( changeType ) {

				case core::componentmodel::topology::ENDING_EVENT:
					{
						//std::cout << "INFO_print : TopologicalMapping - ENDING_EVENT" << std::endl;
						to_tst->getTriangleSetTopologyAlgorithms()->notifyEndingEvent();
						break;
					}

				case core::componentmodel::topology::TRIANGLESREMOVED:
					{
						//std::cout << "INFO_print : TopologicalMapping - TRIANGLESREMOVED" << std::endl;

						unsigned int last;
						unsigned int ind_last;
							
						last= (from_testc->getTetrahedronTriangleShellArray()).size() - 1;

						const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const TrianglesRemoved *>( *itBegin ) )->getArray();
						
                        Triangle tmp;
                        Triangle tmp2;
						unsigned int ind_tmp;

						unsigned int ind_real_last;
						ind_last=(to_tstc->getTriangleArray()).size();

						for (unsigned int i = 0; i <tab.size(); ++i)
						{
							unsigned int k = tab[i];
							unsigned int ind_k;		
							
							std::map<unsigned int, unsigned int>::iterator iter_1 = Glob2LocMap.find(k);
							if(iter_1 != Glob2LocMap.end()) {

								ind_last = ind_last - 1;

								ind_k = Glob2LocMap[k];
								ind_real_last = ind_k;
								
								std::map<unsigned int, unsigned int>::iterator iter_2 = Glob2LocMap.find(last);
								if(iter_2 != Glob2LocMap.end()) {								

									ind_real_last = Glob2LocMap[last]; 																										

									if(k != last){

										Glob2LocMap.erase(Glob2LocMap.find(k));
										Glob2LocMap[k] = ind_real_last;

										Glob2LocMap.erase(Glob2LocMap.find(last));
										Glob2LocMap[last] = ind_k;

										ind_tmp = Loc2GlobVec[ind_real_last];
										Loc2GlobVec[ind_real_last] = Loc2GlobVec[ind_k];  
										Loc2GlobVec[ind_k] = ind_tmp;
									}									
								}

								if( ind_k != ind_last){ 

									Glob2LocMap.erase(Glob2LocMap.find(Loc2GlobVec[ind_last]));
									Glob2LocMap[Loc2GlobVec[ind_last]] = ind_k;
								
									Glob2LocMap.erase(Glob2LocMap.find(Loc2GlobVec[ind_k]));
									Glob2LocMap[Loc2GlobVec[ind_k]] = ind_last;

									ind_tmp = Loc2GlobVec[ind_k];
									Loc2GlobVec[ind_k] = Loc2GlobVec[ind_last];
									Loc2GlobVec[ind_last] = ind_tmp;

								}

								Glob2LocMap.erase(Glob2LocMap.find(Loc2GlobVec[Loc2GlobVec.size() - 1])); 
								Loc2GlobVec.resize( Loc2GlobVec.size() - 1 );
								
								sofa::helper::vector< unsigned int > triangles_to_remove;
								triangles_to_remove.push_back(ind_k);
								to_tst->getTriangleSetTopologyAlgorithms()->removeTriangles(triangles_to_remove, true, false);								

							}else{

								std::cout << "INFO_print : TopologicalMapping - Glob2LocMap should have the visible triangle " << tab[i] << std::endl;
								std::cout << "INFO_print : TopologicalMapping - nb triangles = " << ind_last << std::endl;
							}

							--last;
						}

						break;
					}

				case core::componentmodel::topology::TETRAHEDRAREMOVED:
					{
						//std::cout << "INFO_print : TopologicalMapping - TETRAHEDRAREMOVED" << std::endl;

						if (from_testc) {

							const sofa::helper::vector<Tetrahedron> &tetrahedronArray=from_testc->getTetrahedronArray();

							const sofa::helper::vector<unsigned int> &tab = ( dynamic_cast< const TetrahedraRemoved *>( *itBegin ) )->getArray();

							sofa::helper::vector< Triangle > triangles_to_create;

							for (unsigned int i = 0; i < tab.size(); ++i)
							{

								for (unsigned int j = 0; j < 4; ++j)
								{								
									unsigned int k = (from_testc->getTetrahedronTriangles(tab[i]))[j];

									if (from_testc->getTetrahedronTriangleShell(k).size()==1) { // remove as visible the triangle indexed by k

										// do nothing

									}else{ // from_testc->getTetrahedronTriangleShell(k).size()==2 // add as visible the triangle indexed by k

										unsigned int ind_test;
										if(tab[i] == from_testc->getTetrahedronTriangleShell(k)[0]){

											ind_test = from_testc->getTetrahedronTriangleShell(k)[1];

										}else{ // tab[i] == from_testc->getTetrahedronTriangleShell(k)[1] 

											ind_test = from_testc->getTetrahedronTriangleShell(k)[0];
										}

										bool is_present = false;
										unsigned int k0 = 0;
										while((!is_present) && k0 < i){
											is_present = (ind_test == tab[k0]);
											k0+=1;
										}
										if(!is_present){
                                            
											Triangle t;

											const Tetrahedron &te=tetrahedronArray[ind_test];
											int h = from_testc->getTriangleIndexInTetrahedron(from_testc->getTetrahedronTriangles(ind_test),k);
											
											if (h%2) {
												t[0]=(int)(te[(h+1)%4]); t[1]=(int)(te[(h+2)%4]); t[2]=(int)(te[(h+3)%4]);
											} else {
												t[0]=(int)(te[(h+1)%4]); t[2]=(int)(te[(h+2)%4]); t[1]=(int)(te[(h+3)%4]);
											}
									
											// sort t such that t[0] is the smallest one 
											while ((t[0]>t[1]) || (t[0]>t[2])) {
												int val=t[0]; t[0]=t[1];t[1]=t[2];t[2]=val;
											}																						
																						
											triangles_to_create.push_back(t);																															

											Loc2GlobVec.push_back(k);
											std::map<unsigned int, unsigned int>::iterator iter_1 = Glob2LocMap.find(k);
											if(iter_1 != Glob2LocMap.end() ) {
												std::cout << "INFO_print : TopologicalMapping - fail to add triangle " << k << "which already exists" << std::endl;
												Glob2LocMap.erase(Glob2LocMap.find(k));
											}
											Glob2LocMap[k]=Loc2GlobVec.size()-1;                                            
										}
									}
								}
							}

							to_tstm->addTrianglesProcess(triangles_to_create) ;							

						}						

						break;
					}

					case core::componentmodel::topology::POINTSREMOVED:
					{
						//std::cout << "INFO_print : TopologicalMapping - POINTSREMOVED" << std::endl;
						
						const sofa::helper::vector<unsigned int> tab = ( dynamic_cast< const sofa::component::topology::PointsRemoved * >( *itBegin ) )->getArray();

						sofa::helper::vector<unsigned int> indices;

						for(unsigned int i = 0; i < tab.size(); ++i){

							indices.push_back(tab[i]);
						}

						sofa::helper::vector<unsigned int>& tab_indices = indices;

						to_tstm->removePointsWarning(tab_indices);
						toModel->propagateTopologicalChanges();
						to_tstm->removePointsProcess(tab_indices, false);						

						break;
					}
				
				default:
					// Ignore events that are not Triangle  related.
					break;
				};				

				++itBegin;
			}
			toModel->propagateTopologicalChanges();
		}
	}

	return;
}


} // namespace topology

} // namespace component

} // namespace sofa

#endif
