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
#include <sofa/component/forcefield/TetrahedralQuadraticSpringsForceField.h>
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <sofa/helper/gl/template.h>
#include <sofa/component/topology/TetrahedronData.inl>
#include <sofa/component/topology/TriangleData.inl>
#include <sofa/component/topology/EdgeData.inl>


namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;
using namespace	sofa::component::topology;
using namespace core::componentmodel::topology;

using std::cerr;
using std::cout;
using std::endl;

template< class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::TQSEdgeCreationFunction(int edgeIndex, void* param, EdgeRestInformation &ei,
							const Edge& ,  const helper::vector< unsigned int > &,
							  const helper::vector< double >&)
{
	TetrahedralQuadraticSpringsForceField<DataTypes> *ff= (TetrahedralQuadraticSpringsForceField<DataTypes> *)param;
	if (ff) {
		TetrahedronSetTopology<DataTypes> *_mesh=ff->getTetrahedralTopology();
		assert(_mesh!=0);
		TetrahedronSetGeometryAlgorithms<DataTypes> *ga=_mesh->getTetrahedronSetGeometryAlgorithms();

		// store the rest length of the edge created 
		ei.restLength=ga->computeRestEdgeLength(edgeIndex);
		ei.stiffness=0;
	}
}

template< class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::TQSTriangleCreationFunction(int /*triangleIndex*/, void* param, TriangleRestInformation &tri,
							const Triangle& ,  const helper::vector< unsigned int > &,
							  const helper::vector< double >&)
{
	TetrahedralQuadraticSpringsForceField<DataTypes> *ff= (TetrahedralQuadraticSpringsForceField<DataTypes> *)param;
	if (ff) {
		assert(ff->getTetrahedralTopology()!=0);

		for (unsigned int i=0;i<3;++i)
			tri.angularStiffness[i]=0;
	}
}



template< class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::TQSTetrahedronCreationFunction (int tetrahedronIndex, void* param, 
																					   TetrahedronRestInformation &tinfo,
																					   const Tetrahedron& , 
																					   const helper::vector< unsigned int > &,
																					   const helper::vector< double >&)
{
	TetrahedralQuadraticSpringsForceField<DataTypes> *ff= (TetrahedralQuadraticSpringsForceField<DataTypes> *)param;
	if (ff) {
		TetrahedronSetTopology<DataTypes> *_mesh=ff->getTetrahedralTopology();
		assert(_mesh!=0);
		TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();
		//const std::vector<Edge> &edgeArray=container->getEdgeArray();
		const std::vector< TetrahedronEdges > &tetrahedronEdgeArray=container->getTetrahedronEdgeArray() ;
		const std::vector< TriangleEdges > &triangleEdgeArray=container->getTriangleEdgeArray() ;
		const std::vector< TetrahedronTriangles > &tetrahedronTriangleArray=container->getTetrahedronTriangleArray() ;
		const std::vector< Tetrahedron > &tetrahedronArray=container->getTetrahedronArray() ;
		const std::vector< Triangle > &triangleArray=container->getTriangleArray() ;

		unsigned int j,k,l,triangleIndex,localVertexIndex;

		EdgeData<typename TetrahedralQuadraticSpringsForceField<DataTypes>::EdgeRestInformation> &edgeInfo=ff->getEdgeInfo();
		TriangleData<typename TetrahedralQuadraticSpringsForceField<DataTypes>::TriangleRestInformation> &triangleInfo=ff->getTriangleInfo();

		typename DataTypes::Real volume,restLength[6];
		typename DataTypes::Coord shapeVector[4],point[4];
		const typename DataTypes::VecCoord *restPosition=_mesh->getDOF()->getX0();


		///describe the indices of the 4 tetrahedron vertices  
		const Tetrahedron &t= tetrahedronArray[tetrahedronIndex];
		/// describe the 6 edge indices of created tetrahedron  
		const TetrahedronEdges &te= tetrahedronEdgeArray[tetrahedronIndex];
		/// describe the 4 triangle indices of created tetrahedron  
		const TetrahedronTriangles &tt= tetrahedronTriangleArray[tetrahedronIndex];

		// compute rest area based on Heron's formula
		for(j=0;j<4;++j)
			point[j]=(*restPosition)[t[j]];
		/// compute 6 times the rest volume
		volume=dot(cross(point[1]-point[0],point[2]-point[0]),point[0]-point[3]);
		typename DataTypes::Real lambda=ff->getLambda()*fabs(volume)/12;
		typename DataTypes::Real mu=ff->getMu()*fabs(volume)/12;

		// store shape vectors
		for(j=0;j<4;++j) {
			if ((j%2)==0)
				shapeVector[j]=cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/volume;
			else 
				shapeVector[j]= -cross(point[(j+2)%4] - point[(j+1)%4],point[(j+3)%4] - point[(j+1)%4])/volume;
		}
		/// compute edge stiffness
		for(j=0;j<6;++j) {
			/// local indices of the edge
			k = container->getLocalTetrahedronEdges(j).first;
			l = container->getLocalTetrahedronEdges(j).second;
			restLength[j]=edgeInfo[te[j]].restLength;
			
			tinfo.stiffness[j]=2*restLength[j]*restLength[j]*((lambda+mu)*dot(shapeVector[k],shapeVector[l])*dot(shapeVector[k],shapeVector[l]) +
				mu*shapeVector[k].norm2()*shapeVector[l].norm2());
			edgeInfo[te[j]].stiffness+=tinfo.stiffness[j];
		}
		/// compute triangle stiffness
		for(j=0;j<4;++j){
			triangleIndex=tt[j];
			const TriangleEdges &tre=triangleEdgeArray[triangleIndex];

			tinfo.angularStiffness[3*j]=(lambda+mu)*dot(shapeVector[(j+1)%4],shapeVector[(j+2)%4])*dot(shapeVector[(j+1)%4],shapeVector[(j+3)%4]) +
				mu*shapeVector[(j+1)%4].norm2()*dot(shapeVector[(j+2)%4],shapeVector[(j+3)%4]);
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+1)%4]);
			tinfo.angularStiffness[3*j]*=2*edgeInfo[tre[(localVertexIndex+1)%3]].restLength*
				edgeInfo[tre[(localVertexIndex+2)%3]].restLength;
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]+=tinfo.angularStiffness[3*j];

			tinfo.angularStiffness[3*j+1]=(lambda+mu)*dot(shapeVector[(j+2)%4],shapeVector[(j+3)%4])*dot(shapeVector[(j+2)%4],shapeVector[(j+1)%4]) +
				mu*shapeVector[(j+2)%4].norm2()*dot(shapeVector[(j+1)%4],shapeVector[(j+3)%4]);
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+2)%4]);
			tinfo.angularStiffness[3*j+1]*=2*edgeInfo[tre[(localVertexIndex+1)%3]].restLength*
				edgeInfo[tre[(localVertexIndex+2)%3]].restLength;
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]+=tinfo.angularStiffness[3*j+1];

			tinfo.angularStiffness[3*j+2]=(lambda+mu)*dot(shapeVector[(j+3)%4],shapeVector[(j+1)%4])*dot(shapeVector[(j+3)%4],shapeVector[(j+2)%4]) +
				mu*shapeVector[(j+3)%4].norm2()*dot(shapeVector[(j+1)%4],shapeVector[(j+2)%4]);
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+3)%4]);
			tinfo.angularStiffness[3*j+2]*=2*edgeInfo[tre[(localVertexIndex+1)%3]].restLength*
				edgeInfo[tre[(localVertexIndex+2)%3]].restLength;
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]+=tinfo.angularStiffness[3*j+2];
		}
		/// compute volumetric stiffness
		for(j=1;j<4;++j){
			k=j%3+1;
			l=(j+1)%3+1;
			
			tinfo.volumetricStiffness[j-1]=lambda*dot(shapeVector[0],shapeVector[j])*dot(shapeVector[k],shapeVector[l]) +
				mu*(dot(shapeVector[0],shapeVector[k])*dot(shapeVector[j],shapeVector[l])+dot(shapeVector[0],shapeVector[l])*dot(shapeVector[j],shapeVector[k]));		
			tinfo.volumetricStiffness[j-1]*=2*restLength[j-1]*restLength[6-j];
		}

	}

} 


template< class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::TQSTetrahedronDestroyFunction(int tetrahedronIndex, void* param, typename TetrahedralQuadraticSpringsForceField<DataTypes>::TetrahedronRestInformation &tinfo)
{
	TetrahedralQuadraticSpringsForceField<DataTypes> *ff= (TetrahedralQuadraticSpringsForceField<DataTypes> *)param;
	if (ff) {
		TetrahedronSetTopology<DataTypes> *_mesh=ff->getTetrahedralTopology();
		assert(_mesh!=0);
		TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();
		const std::vector< TetrahedronEdges > &tetrahedronEdgeArray=container->getTetrahedronEdgeArray() ;
		const std::vector< TetrahedronTriangles > &tetrahedronTriangleArray=container->getTetrahedronTriangleArray() ;
		const std::vector< Triangle > &triangleArray=container->getTriangleArray() ;
		const std::vector< Tetrahedron > &tetrahedronArray=container->getTetrahedronArray() ;

		unsigned int j,triangleIndex,localVertexIndex;
		
		EdgeData<typename TetrahedralQuadraticSpringsForceField<DataTypes>::EdgeRestInformation> &edgeInfo=ff->getEdgeInfo();
		TriangleData<typename TetrahedralQuadraticSpringsForceField<DataTypes>::TriangleRestInformation> &triangleInfo=ff->getTriangleInfo();

		///describe the indices of the 4 tetrahedron vertices  
		const Tetrahedron &t= tetrahedronArray[tetrahedronIndex];
		/// describe the jth edge index of triangle no i 
		const TetrahedronEdges &te= tetrahedronEdgeArray[tetrahedronIndex];
		/// describe the 4 triangle indices of created tetrahedron  
		const TetrahedronTriangles &tt= tetrahedronTriangleArray[tetrahedronIndex];

		// update edge stiffness
		for(j=0;j<6;++j) {
			edgeInfo[te[j]].stiffness -= tinfo.stiffness[j]; 
		}
		/// update triangle stiffness
		for(j=0;j<4;++j){
			triangleIndex=tt[j];
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+1)%4]);
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]-=tinfo.angularStiffness[3*j];
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+2)%4]);
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]-=tinfo.angularStiffness[3*j+1];
			localVertexIndex=container->getVertexIndexInTriangle(triangleArray[triangleIndex],t[(j+3)%4]);
			triangleInfo[triangleIndex].angularStiffness[localVertexIndex]-=tinfo.angularStiffness[3*j+2];
		}


	}
} 

template <class DataTypes> TetrahedralQuadraticSpringsForceField<DataTypes>::TetrahedralQuadraticSpringsForceField() 
: _mesh(NULL)
, _initialPoints(0)
, updateMatrix(true)
, f_poissonRatio(initData(&f_poissonRatio,(Real)0.3,"poissonRatio","Poisson ratio in Hooke's law"))
, f_youngModulus(initData(&f_youngModulus,(Real)1000.,"youngModulus","Young modulus in Hooke's law"))
, f_useAngularSprings(initData(&f_useAngularSprings,true,"useAngularSprings","If Angular Springs should be used or not"))
, lambda(0)
, mu(0)
	{
	}

template <class DataTypes> void TetrahedralQuadraticSpringsForceField<DataTypes>::handleTopologyChange()
{
	sofa::core::componentmodel::topology::BaseTopology *topology = static_cast<sofa::core::componentmodel::topology::BaseTopology *>(getContext()->getMainTopology());

	std::list<const TopologyChange *>::const_iterator itBegin=topology->firstChange();
	std::list<const TopologyChange *>::const_iterator itEnd=topology->lastChange();

	edgeInfo.handleTopologyEvents(itBegin,itEnd);
	triangleInfo.handleTopologyEvents(itBegin,itEnd);
	tetrahedronInfo.handleTopologyEvents(itBegin,itEnd);

}

template <class DataTypes> TetrahedralQuadraticSpringsForceField<DataTypes>::~TetrahedralQuadraticSpringsForceField()
{

}

template <class DataTypes> void TetrahedralQuadraticSpringsForceField<DataTypes>::init()
{
	std::cerr << "initializing TetrahedralQuadraticSpringsForceField" << std::endl;
	this->Inherited::init();
	_mesh =0;
	if (getContext()->getMainTopology()!=0)
		_mesh= dynamic_cast<TetrahedronSetTopology<DataTypes>*>(getContext()->getMainTopology());

	if ((_mesh==0) || (_mesh->getTetrahedronSetTopologyContainer()->getNumberOfTetrahedra()==0))
	{
		std::cerr << "ERROR(TetrahedralQuadraticSpringsForceField): object must have a Tetrahedral Set Topology.\n";
		return;
	}
	updateLameCoefficients();

	TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();

/// prepare to store info in the triangle array
	tetrahedronInfo.resize(container->getNumberOfTetrahedra());
	/// prepare to store info in the triangle array
	triangleInfo.resize(container->getNumberOfTriangles());
	/// prepare to store info in the edge array
	edgeInfo.resize(container->getNumberOfEdges());

    // get restPosition
	if (_initialPoints.size() == 0)
	{
		const VecCoord& p = *this->mstate->getX0();
		_initialPoints=p;
	}
	unsigned int i;
	const std::vector<Edge> &edgeArray=container->getEdgeArray();
	for (i=0;i<container->getNumberOfEdges();++i) {
		TQSEdgeCreationFunction(i, (void*) this, edgeInfo[i],
			edgeArray[i],  (const std::vector< unsigned int > )0,
			(const std::vector< double >)0);
	}
	const std::vector<Triangle> &triangleArray=container->getTriangleArray();
	for (i=0;i<container->getNumberOfTriangles();++i) {
		TQSTriangleCreationFunction(i, (void*) this, triangleInfo[i],
			triangleArray[i],  (const std::vector< unsigned int > )0,
			(const std::vector< double >)0);
	}

	const std::vector<Tetrahedron> &tetrahedronArray=container->getTetrahedronArray();
	for (i=0;i<container->getNumberOfTetrahedra();++i) {
		TQSTetrahedronCreationFunction(i, (void*) this, tetrahedronInfo[i],
			tetrahedronArray[i],  (const std::vector< unsigned int > )0,
			(const std::vector< double >)0);
	}

	edgeInfo.setCreateFunction(TQSEdgeCreationFunction);
	triangleInfo.setCreateFunction(TQSTriangleCreationFunction);
	tetrahedronInfo.setCreateFunction(TQSTetrahedronCreationFunction);
	tetrahedronInfo.setDestroyFunction(TQSTetrahedronDestroyFunction);
	edgeInfo.setCreateParameter( (void *) this );
	edgeInfo.setDestroyParameter( (void *) this );
	triangleInfo.setCreateParameter( (void *) this );
	triangleInfo.setDestroyParameter( (void *) this );
	tetrahedronInfo.setCreateParameter( (void *) this );
	tetrahedronInfo.setDestroyParameter( (void *) this );

}


template <class DataTypes> 
double TetrahedralQuadraticSpringsForceField<DataTypes>::getPotentialEnergy(const VecCoord& /*x*/)
{
	std::cerr<<"TetrahedralQuadraticSpringsForceField::getPotentialEnergy-not-implemented !!!"<<endl;
    return 0;
}

template <class DataTypes> 
void TetrahedralQuadraticSpringsForceField<DataTypes>::addForce(VecDeriv& f, const VecCoord& x, const VecDeriv& v)
{
	unsigned int i,j,k,l,v0,v1;
	TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();
	unsigned int nbEdges=container->getNumberOfEdges();
	unsigned int nbTriangles=container->getNumberOfTriangles();
	unsigned int nbTetrahedra=container->getNumberOfTetrahedra();
	const std::vector<Edge> &edgeArray=container->getEdgeArray();
	const std::vector< TriangleEdges > &triangleEdgeArray=container->getTriangleEdgeArray() ;
	const std::vector< TetrahedronEdges > &tetrahedronEdgeArray=container->getTetrahedronEdgeArray();
	const std::vector< Triangle> &triangleArray=container->getTriangleArray() ;
	const std::vector< Tetrahedron> &tetrahedronArray=container->getTetrahedronArray() ;


	Real val,L;
	TriangleRestInformation *tinfo;
	EdgeRestInformation *einfo;
	TetrahedronRestInformation *tetinfo;

	assert(this->mstate);

	Deriv force;
	Coord dp,dv;



	for(i=0; i<nbEdges; i++ )
	{
		einfo=&edgeInfo[i];
		v0=edgeArray[i].first;
		v1=edgeArray[i].second;
		dp=x[v0]-x[v1];
		dv=v[v0]-v[v1];
		L=einfo->currentLength=dp.norm();
		einfo->dl=einfo->currentLength-einfo->restLength;

		val=einfo->stiffness*einfo->dl/L;
		force=dp*val;
		f[v1]+=force;
		f[v0]-=force;
	}
	if (f_useAngularSprings.getValue()==true) {
		for(i=0; i<nbTriangles; i++ )
		{
			tinfo=&triangleInfo[i];
			/// describe the jth edge index of triangle no i 
			const TriangleEdges &tea= triangleEdgeArray[i];
			/// describe the jth vertex index of triangle no i 
			const Triangle &ta= triangleArray[i];

			// compute the force from angular springs
			for(j=0;j<3;++j) {
				k=(j+1)%3;
				l=(j+2)%3;
				force=(x[ta[k]] - x[ta[l]])*
					(edgeInfo[tea[k]].dl * tinfo->angularStiffness[l] +edgeInfo[tea[l]].dl * tinfo->angularStiffness[k])/edgeInfo[tea[j]].currentLength;
				f[ta[l]]+=force;
				f[ta[k]]-=force;			
			}
		}
		for(i=0; i<nbTetrahedra; i++ )
		{
			tetinfo=&tetrahedronInfo[i];
			/// describe the jth edge index of triangle no i 
			const TetrahedronEdges &tea= tetrahedronEdgeArray[i];
			/// describe the jth vertex index of triangle no i 
			const Tetrahedron &ta= tetrahedronArray[i];



			force=(x[ta[1]] - x[ta[0]])*edgeInfo[tea[5]].dl*tetinfo->volumetricStiffness[0]/edgeInfo[tea[0]].currentLength;
			f[ta[0]]+=force;
			f[ta[1]]-=force;			

			force=(x[ta[3]] - x[ta[2]])*edgeInfo[tea[0]].dl*tetinfo->volumetricStiffness[0]/edgeInfo[tea[5]].currentLength;
			f[ta[2]]+=force;
			f[ta[3]]-=force;

			force=(x[ta[2]] - x[ta[0]])*edgeInfo[tea[4]].dl*tetinfo->volumetricStiffness[1]/edgeInfo[tea[1]].currentLength;
			f[ta[0]]+=force;
			f[ta[2]]-=force;

			force=(x[ta[3]] - x[ta[1]])*edgeInfo[tea[1]].dl*tetinfo->volumetricStiffness[1]/edgeInfo[tea[4]].currentLength;
			f[ta[1]]+=force;
			f[ta[3]]-=force;

			force=(x[ta[3]] - x[ta[0]])*edgeInfo[tea[3]].dl*tetinfo->volumetricStiffness[2]/edgeInfo[tea[2]].currentLength;
			f[ta[0]]+=force;
			f[ta[3]]-=force;

			force=(x[ta[2]] - x[ta[1]])*edgeInfo[tea[2]].dl*tetinfo->volumetricStiffness[2]/edgeInfo[tea[3]].currentLength;
			f[ta[1]]+=force;
			f[ta[2]]-=force;

		}


	
	//	std::cerr << "tinfo->gamma[0] "<<tinfo->gamma[0]<<std::endl;

	}
	
	updateMatrix=true;
}


template <class DataTypes> 
void TetrahedralQuadraticSpringsForceField<DataTypes>::addDForce(VecDeriv& df, const VecDeriv& dx)
{
	unsigned int i,j,k,l;
	TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();
	unsigned int nbEdges=container->getNumberOfEdges();
	const std::vector< Edge> &edgeArray=container->getEdgeArray() ;

	EdgeRestInformation *einfo;

//	std::cerr << "start addDForce" << std::endl;


	assert(this->mstate);
	VecDeriv& x = *this->mstate->getX();


	Deriv deltax,res;

	if (updateMatrix) {
		int u,v;
		Real val1,val2,vali,valj,valk;
		Coord dpj,dpk,dpi;
		TriangleRestInformation *trinfo;
		TetrahedronRestInformation *tinfo;



	//	std::cerr <<"updating matrix"<<std::endl;
		updateMatrix=false;

		/// store edge stifness matrix
		for(l=0; l<nbEdges; l++) {
			const Edge &e=edgeArray[l];
			einfo=&edgeInfo[l];
			dpk=x[e.first]-x[e.second];
			Mat3 &m=einfo->DfDx;
			val1 = -einfo->stiffness*einfo->dl;
			val1/=einfo->currentLength;
			val2= -einfo->stiffness*einfo->restLength;
			val2/=einfo->currentLength*einfo->currentLength*einfo->currentLength;

			for (u=0;u<3;++u) {
				for (v=0;v<3;++v) {
					m[u][v]=dpk[u]*dpk[v]*val2;
				}
				m[u][u]+=val1;
			}
		}

		if (f_useAngularSprings.getValue()==true) {

			/// add triangle stifness matrix
			unsigned int nbTriangles=container->getNumberOfTriangles();
			const std::vector< TriangleEdges > &triangleEdgeArray=container->getTriangleEdgeArray() ;
			const std::vector< Triangle> &triangleArray=container->getTriangleArray() ;

			for(l=0; l<nbTriangles; l++ )
			{
				/// describe the jth edge index of triangle no i 
				const TriangleEdges &tea= triangleEdgeArray[l];
				/// describe the jth vertex index of triangle no i 
				const Triangle &ta= triangleArray[l];
				trinfo=&triangleInfo[l];


				// for all triangle vertex
				for(k=0;k<3;++k) {
					i=(k+1)%3;
					j=(k+2)%3;
					// eventually swap i and j to follow the order of the edge
					if (ta[i]!=edgeArray[tea[k]].first) {
						u=j;j=i;i=u;
					}
					einfo=&edgeInfo[tea[k]];
					Mat3 &m=einfo->DfDx;
					dpk = x[ta[i]]- x[ta[j]];

					dpj = x[ta[i]]- x[ta[k]];
					dpi = x[ta[j]]- x[ta[k]];

					val1 = -(trinfo->angularStiffness[i]*edgeInfo[tea[j]].dl+
						trinfo->angularStiffness[j]*edgeInfo[tea[i]].dl);
					val2 = -val1;
					val1/=edgeInfo[tea[k]].currentLength;
					val2/=edgeInfo[tea[k]].currentLength*edgeInfo[tea[k]].currentLength*edgeInfo[tea[k]].currentLength;

					valk=trinfo->angularStiffness[k]/(edgeInfo[tea[j]].currentLength*
						edgeInfo[tea[i]].currentLength);
					vali=trinfo->angularStiffness[i]/(edgeInfo[tea[j]].currentLength*
						edgeInfo[tea[k]].currentLength);
					valj=trinfo->angularStiffness[j]/(edgeInfo[tea[k]].currentLength*
						edgeInfo[tea[i]].currentLength);

					for (u=0;u<3;++u) {
						for (v=0;v<3;++v) {
							m[u][v]+=dpk[u]*dpk[v]*val2
								+dpj[u]*dpi[v]*valk
								-dpj[u]*dpk[v]*vali
								+dpk[u]*dpi[v]*valj;
						}
						m[u][u]+=val1;
					}
				}
			}

			/// add tetrahedron stifness matrix
			unsigned int nbTetrahedra=container->getNumberOfTetrahedra();
			const std::vector< TetrahedronEdges > &tetrahedronEdgeArray=container->getTetrahedronEdgeArray() ;
			const std::vector< Tetrahedron> &tetrahedronArray=container->getTetrahedronArray() ;
			const unsigned int edgePairIndex[4][4]={{4,0,1,2},{0,4,2,1},{1,2,4,0},{2,1,0,4}};
			const unsigned int edgeIndex[4][4]={{7,0,1,2},{0,7,3,4},{1,3,7,5},{2,4,5,7}};


			unsigned int edgeIndex1,edgeIndex2,m,n,localIndexi,localIndexk,localIndexl,localIndexj;
			EdgeRestInformation *einfo1,*einfo2;
			Real valik,valil,val1,val2,valij,valkl;
			Coord dpki,dpjl,dpli,dpjk,dpji,dpkl;


			for(m=0; m<nbTetrahedra; m++ )
			{
				/// describe the jth edge index of triangle no i 
				const TetrahedronEdges &tea= tetrahedronEdgeArray[m];
				/// describe the jth vertex index of triangle no i 
				const Tetrahedron &ta= tetrahedronArray[m];
				// for all tetrahedron opposite edge pair
				tinfo=&tetrahedronInfo[m];

				for(n=0;n<3;++n) {
					// by construction those 2 edges are opposite to each other
					edgeIndex1 = n;
					edgeIndex2 = 5-n;
					einfo1=&edgeInfo[tea[edgeIndex1]];
					einfo2=&edgeInfo[tea[edgeIndex2]];


					i=edgeArray[tea[edgeIndex1]].first;
					j=edgeArray[tea[edgeIndex1]].second;
					k = edgeArray[tea[edgeIndex2]].first;
					l = edgeArray[tea[edgeIndex2]].second;

					localIndexi=container->getVertexIndexInTetrahedron(ta,i);
					localIndexk=container->getVertexIndexInTetrahedron(ta,k);
					localIndexl=container->getVertexIndexInTetrahedron(ta,l);
					localIndexj=container->getVertexIndexInTetrahedron(ta,j);


					Mat3 &m1=einfo1->DfDx;
					Mat3 &m2=einfo2->DfDx;

					val1 = -tinfo->volumetricStiffness[n]*einfo2->dl/einfo1->currentLength;
					val2 = -tinfo->volumetricStiffness[n]*einfo1->dl/einfo2->currentLength;
					valij= tinfo->volumetricStiffness[n]*einfo2->dl/(einfo1->currentLength*einfo1->currentLength*einfo1->currentLength);
					valkl= tinfo->volumetricStiffness[n]*einfo1->dl/(einfo2->currentLength*einfo2->currentLength*einfo2->currentLength);
					valik= -tinfo->volumetricStiffness[edgePairIndex[localIndexi][localIndexk]]/(edgeInfo[tea[edgeIndex[localIndexi][localIndexk]]].currentLength*edgeInfo[tea[edgeIndex[localIndexj][localIndexl]]].currentLength);
					valil= -tinfo->volumetricStiffness[edgePairIndex[localIndexi][localIndexl]]/(edgeInfo[tea[edgeIndex[localIndexi][localIndexl]]].currentLength*edgeInfo[tea[edgeIndex[localIndexk][localIndexj]]].currentLength);

					dpki=x[k]-x[i];
					dpjl=x[j]-x[l];
					dpli=x[l]-x[i];
					dpjk=x[j]-x[k];
					dpji=x[j]-x[i];
					dpkl=x[k]-x[l];



					for (u=0;u<3;++u) {
						for (v=0;v<3;++v) {
							m1[u][v]+=dpji[u]*dpji[v]*valij
								+dpki[u]*dpjl[v]*valik
								+dpli[u]*dpjk[v]*valil;
							m2[u][v]+=dpkl[u]*dpkl[v]*valkl
								+dpki[u]*dpjl[v]*valik
								+dpli[v]*dpjk[u]*valil;
						}
						m1[u][u]+=val1;
						m2[u][u]+=val2;
					}

				}

			}

		}
	} 

	unsigned int v0,v1;
	for(l=0; l<nbEdges; l++ )
		{
			einfo=&edgeInfo[l];
			v0=edgeArray[l].first;
			v1=edgeArray[l].second;

			deltax= dx[v0] -dx[v1];
			res=einfo->DfDx*deltax;
			df[v0]+=res;
			df[v1]-= einfo->DfDx.transposeMultiply(deltax);

		}
}


template<class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::updateLameCoefficients()
{
	lambda= f_youngModulus.getValue()*f_poissonRatio.getValue()/((1-2*f_poissonRatio.getValue())*(1+f_poissonRatio.getValue()));
	mu = f_youngModulus.getValue()/(2*(1+f_poissonRatio.getValue()));
//	std::cerr << "initialized Lame coef : lambda=" <<lambda<< " mu="<<mu<<std::endl;
}


template<class DataTypes>
void TetrahedralQuadraticSpringsForceField<DataTypes>::draw()
{
//	unsigned int i;
	if (!getContext()->getShowForceFields()) return;
	if (!this->mstate) return;

	if (getContext()->getShowWireFrame())
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

// 	VecCoord& x = *this->mstate->getX();
// 	TetrahedronSetTopologyContainer *container=_mesh->getTetrahedronSetTopologyContainer();
	/*
	unsigned int nbTriangles=container->getNumberOfTriangles();
	const std::vector< Triangle> &triangleArray=container->getTriangleArray() ;


	glDisable(GL_LIGHTING);

	glBegin(GL_TRIANGLES);
	for(i=0;i<nbTriangles; ++i)
	{
		int a = triangleArray[i][0];
		int b = triangleArray[i][1];
		int c = triangleArray[i][2];

		glColor4f(0,1,0,1);
		helper::gl::glVertexT(x[a]);
		glColor4f(0,0.5,0.5,1);
		helper::gl::glVertexT(x[b]);
		glColor4f(0,0,1,1);
		helper::gl::glVertexT(x[c]);
	}
	glEnd();
*/

	if (getContext()->getShowWireFrame())
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		
}

} // namespace forcefield

} // namespace Components

} // namespace Sofa
