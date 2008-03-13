#ifndef SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_INL

#include <sofa/component/forcefield/SpringEdgeDataForceField.h>
#include <sofa/helper/io/MassSpringLoader.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/component/topology/TopologyChangedEvent.h>
#include <sofa/helper/system/config.h>
#include <assert.h>
#include <iostream>
#include <GL/gl.h>
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::core::componentmodel::behavior;

template<class DataTypes>
void springCreationFunction(int index, 
							void* param, typename SpringEdgeDataForceField<DataTypes>::Spring& t, 
							const component::topology::Edge& ,  
							const std::vector< unsigned int > &ancestors,
							const std::vector< double >& coefs)
{
	SpringEdgeDataForceField<DataTypes> *ff= static_cast<SpringEdgeDataForceField<DataTypes> *>(param);
	if (ff) {
		sofa::component::topology::EdgeSetTopology<DataTypes>* topology = dynamic_cast<sofa::component::topology::EdgeSetTopology<DataTypes>*>(ff->getContext()->getMainTopology());
		if (topology) {
			component::topology::EdgeSetGeometryAlgorithms<DataTypes> *ga=topology->getEdgeSetGeometryAlgorithms();
			t.restLength=ga->computeRestEdgeLength(index);
			if 	((ancestors!= (const std::vector< unsigned int >)0) &&  (ancestors.size()>0)) {
				t.kd=t.ks=0;
				const component::topology::EdgeData<typename SpringEdgeDataForceField<DataTypes>::Spring> &sa=ff->getSpringArray();
				unsigned int i;
				for (i=0;i<ancestors.size();++i) {
					t.kd+=(typename DataTypes::Real)( sa[i].kd*coefs[i]);
					t.ks+=(typename DataTypes::Real)(sa[i].ks*coefs[i]);
				}
			} else {
			t.kd=ff->getStiffness();
			t.ks=ff->getViscosity();
			}
		}
	}
}
template <class DataTypes>
class SpringEdgeDataForceField<DataTypes>::Loader : public helper::io::MassSpringLoader
{
	public:
		SpringEdgeDataForceField<DataTypes>* dest;
		Loader(SpringEdgeDataForceField<DataTypes>* dest) : dest(dest) {}
		virtual void addSpring(int m1, int m2, double ks, double kd, double initpos)
		{
			dest->addSpring(m1,m2,ks,kd,initpos);
		}
		virtual void setNumSprings(int n) {
		dest->resizeArray((unsigned int )n);
		}

};

template <class DataTypes>
bool SpringEdgeDataForceField<DataTypes>::load(const char *filename)
{
	if (filename && filename[0])
	{
		Loader loader(this);
		return loader.load(filename);
	}
	else return false;
}

template <class DataTypes>
void SpringEdgeDataForceField<DataTypes>::resizeArray(unsigned int n) 
{
	springArray.resize(n);
}
template <class DataTypes>
void SpringEdgeDataForceField<DataTypes>::addSpring(int m1, int m2, double ks, double kd, double initpos)
{
	sofa::component::topology::EdgeSetTopology<DataTypes> *topology = dynamic_cast<sofa::component::topology::EdgeSetTopology<DataTypes>*>(object->getContext()->getMainTopology());
	component::topology::EdgeSetTopologyContainer *container=topology->getEdgeSetTopologyContainer();

	int e=container->getEdgeIndex((unsigned int)m1,(unsigned int)m2);
	if (e>=0)
		springArray[e]=Spring((Real)ks,(Real)kd,(Real)initpos);
}
template <class DataTypes>
SpringEdgeDataForceField<DataTypes>::SpringEdgeDataForceField(core::componentmodel::behavior::MechanicalState<DataTypes>* _object)
	: core::componentmodel::behavior::ForceField<DataTypes>(_object),object(_object),
	m_filename( dataField(&m_filename,std::string("untitled"),"filename","File name from which the spring informations are loaded") ),
    m_stiffness( dataField(&m_stiffness,1.0,"stiffness","Default edge stiffness used in absence of file information") ),
    m_viscosity( dataField(&m_viscosity,1.0,"viscosity","Default edge viscosity used in absence of file information") )
	{
		springArray.setCreateFunction(springCreationFunction<DataTypes>);
		springArray.setCreateParameter( (void *) this );
	}
	

template <class DataTypes>
void SpringEdgeDataForceField<DataTypes>::init()
{
	ForceField<DataTypes>::init();
	topology = dynamic_cast<sofa::component::topology::EdgeSetTopology<DataTypes>*>(getContext()->getMainTopology());
	assert(topology!=0);
	/// check that the filename as changed
	if (m_filename.getValue()!="untitled")
		// load the springs from a file
		load(( const char *)(m_filename.getValue().c_str()));
	else {
		// create springs based on the mesh topology
		createDefaultSprings();
	}
	f_listening.setValue(true);
}
template < class T , class DataTypes>
class EdgeLengthArrayInterface: public component::topology::BasicArrayInterface<T> {
protected:
	 component::topology::EdgeData<typename SpringEdgeDataForceField<DataTypes>::Spring> &springArray;
public:
	EdgeLengthArrayInterface( component::topology::EdgeData<typename SpringEdgeDataForceField<DataTypes>::Spring> &_sa) : springArray(_sa) {
	}
	// Access to i-th element.
	virtual T & operator[](int i)
	{
		return springArray[i].restLength;
	}
};

template <class DataTypes>
void SpringEdgeDataForceField<DataTypes>::createDefaultSprings() 
{
	component::topology::EdgeSetTopologyContainer *container=topology->getEdgeSetTopologyContainer();
	const std::vector<component::topology::Edge> &ea=container->getEdgeArray();
	springArray.resize(ea.size());
	component::topology::EdgeSetGeometryAlgorithms<DataTypes> *ga=topology->getEdgeSetGeometryAlgorithms();
	EdgeLengthArrayInterface<Real,DataTypes> elai(springArray);
	ga->computeEdgeLength(elai);
	unsigned int i;
	for (i=0;i<ea.size();++i) {
		springArray[i].ks=(Real)m_stiffness.getValue();
		springArray[i].kd=(Real)m_viscosity.getValue();
	}

}
template<class DataTypes>
void SpringEdgeDataForceField<DataTypes>::handleEvent( Event* e )
{
    if( KeypressedEvent* ke = dynamic_cast<KeypressedEvent*>( e ) )
    {
		/// handle ctrl+d key
		if (ke->getKey()=='D') {
			if (topology->getEdgeSetTopologyContainer()->getNumberOfEdges()>12) {
				component::topology::EdgeSetTopologyAlgorithms<DataTypes> *esta=topology->getEdgeSetTopologyAlgorithms();
				std::vector<unsigned int> edgeArray;
				edgeArray.push_back(12);
				esta->removeEdges(edgeArray);
			}
//			esta->splitEdges(edgeArray);
		}
	} else {
		component::topology::TopologyChangedEvent *tce=dynamic_cast<component::topology::TopologyChangedEvent *>(e);
		/// test that the event is a change of topology and that it 
		if ((tce) && (tce->getTopology()== getContext()->getMainTopology())) {
			core::componentmodel::topology::BaseTopology *topology = static_cast<core::componentmodel::topology::BaseTopology *>(getContext()->getMainTopology());

			std::list<const core::componentmodel::topology::TopologyChange *>::const_iterator itBegin=topology->firstChange();
			std::list<const core::componentmodel::topology::TopologyChange *>::const_iterator itEnd=topology->lastChange();
			/// Topological events are handled by the EdgeData structure
			springArray.handleTopologyEvents(itBegin,itEnd);
		}
	}

}
template<class DataTypes>
void SpringEdgeDataForceField<DataTypes>::addForce(VecDeriv& f, const VecCoord& p, const VecDeriv& v)
{
	assert(this->object);
	m_potentialEnergy = 0;
	component::topology::EdgeSetTopologyContainer *container=topology->getEdgeSetTopologyContainer();
	const std::vector<component::topology::Edge> &ea=container->getEdgeArray();
	Coord u;
	Real d,inverseLength,elongation,elongationVelocity,forceIntensity;
	
	Deriv relativeVelocity,force;
	for (unsigned int i=0; i<ea.size(); i++)
	{
		const component::topology::Edge &e=ea[i];
		const Spring &s=springArray[i];
		u = p[e.second]-p[e.first];
		d = u.norm();
		inverseLength = 1.0f/d;
		u *= inverseLength;
		elongation = (Real)(d - s.restLength);
		m_potentialEnergy += elongation * elongation * s.ks /2;
		relativeVelocity = v[e.second]-v[e.first];
		elongationVelocity = dot(u,relativeVelocity);
		forceIntensity = (Real)(s.ks*elongation+s.kd*elongationVelocity);
		force = u*forceIntensity;
		f[e.first]+=force;
		f[e.second]-=force;
	}
	updateMatrix=true;
}

template<class DataTypes>
void SpringEdgeDataForceField<DataTypes>::addDForce(VecDeriv& df, const VecDeriv& dx)
{
	assert(this->object);
	const VecCoord& p = *object->getX();
	const VecDeriv& v = *object->getV();
	component::topology::EdgeSetTopologyContainer *container=topology->getEdgeSetTopologyContainer();
	const std::vector<component::topology::Edge> &ea=container->getEdgeArray();
	Deriv dforce,d;
	unsigned int i;



	if (updateMatrix==true) {
		updateMatrix=false;
		int j,k;
		Real d,inverseLength,elongation,elongationVelocity,forceIntensity,tgt;
		Deriv relativeVelocity;
		Coord u;


		for (i=0; i<ea.size(); i++)
		{
			const component::topology::Edge &e=ea[i];
			Spring &s=springArray[i];
			u = p[e.second]-p[e.first];
			d = u.norm();
			inverseLength = 1.0f/d;
			u *= inverseLength;
			elongation = (Real)(d - s.restLength);
			relativeVelocity = v[e.second]-v[e.first];
			elongationVelocity = dot(u,relativeVelocity);
			forceIntensity = (Real)(s.ks*elongation+s.kd*elongationVelocity);
			tgt = forceIntensity * inverseLength;
			for(  j=0; j<3; ++j )
			{
				for(  k=0; k<3; ++k )
				{
					s.dfdx[j][k] = ((Real)s.ks-tgt) * u[j] * u[k];
				}
				s.dfdx[j][j] += tgt;
			}
		}
	}

	for ( i=0; i<ea.size(); i++)
	{
		const component::topology::Edge &e=ea[i];
		const Spring &s=springArray[i];
		d = dx[e.second]-dx[e.first];
		dforce = (s.dfdx)*d;
		df[e.first]+=dforce;
		df[e.second]-=dforce;
	}

}

template<class DataTypes>
void SpringEdgeDataForceField<DataTypes>::draw()
{
	if (getContext()->getShowForceFields()==false) 
		return;
	const VecCoord& p = *this->object->getX();
	component::topology::EdgeSetTopologyContainer *container=topology->getEdgeSetTopologyContainer();
	const std::vector<component::topology::Edge> &ea=container->getEdgeArray();

    glDisable(GL_LIGHTING);
	
	glBegin(GL_LINES);
	for (unsigned int i=0; i<springArray.size(); i++)
	{
		const component::topology::Edge &e=ea[i];
		const Spring &s=springArray[i];

		Real d = (p[e.second]-p[e.first]).norm();

		if (d<s.restLength*0.9999)
			glColor4f(1,0.5f,0,1);
		else
			glColor4f(0,1,0.5f,1);

		glVertex3d(p[e.first][0],p[e.first][1],p[e.first][2]);
		glVertex3d(p[e.second][0],p[e.second][1],p[e.second][2]);
	}
	glEnd();
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
