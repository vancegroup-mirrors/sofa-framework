#ifndef SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_H

#include <sofa/core/componentmodel/behavior/ForceField.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/VisualModel.h>
#include <sofa/component/topology/EdgeData.h>
#include <sofa/component/topology/EdgeData.inl>
#include <sofa/defaulttype/Vec.h>
#include <vector>


namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

//using namespace sofa::core::objectmodel;

template<class DataTypes>
class SpringEdgeDataForceField : public core::componentmodel::behavior::ForceField<DataTypes>, public core::VisualModel
{
public:
	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename Coord::value_type Real;
	class Mat3 : public fixed_array<Deriv,3>
	{
	public:
		Deriv operator*(const Deriv& v) const
		{
			return Deriv((*this)[0]*v,(*this)[1]*v,(*this)[2]*v);
		}
	};

	struct Spring {
		Real  ks;			// spring stiffness
		Real  kd;			// damping factor 
		Real  restLength;	// rest length of the spring 
		Mat3  dfdx; // the edge stiffness matrix
		
		Spring(Real _ks, Real _kd, Real _rl) : ks(_ks), kd(_kd), restLength(_rl) {
		}
		Spring() : ks(1.0),kd(1.0),restLength(0.0) {
		}
	};
protected:
	double m_potentialEnergy;
	/// if the edge matrices should be updated
	bool updateMatrix;
	/// where the springs information are stored
	component::topology::EdgeData<Spring> springArray;
	core::componentmodel::behavior::MechanicalState<DataTypes>* object;
	/// the EdgeSet topology used to get the list of edges
	component::topology::EdgeSetTopology<DataTypes> *topology;
	/// the filename where to load the spring information
	DataField<std::string> m_filename;
	/// By default, assume that all edges have the same stiffness
	DataField<double> m_stiffness;
	/// By default, assume that all edges have the same viscosity
	DataField<double> m_viscosity;

	//void addSpringForce(double& potentialEnergy, VecDeriv& f1, const VecCoord& p1, const VecDeriv& v1, VecDeriv& f2, const VecCoord& p2, const VecDeriv& v2, int i, const Spring& spring);
	void addSpring(int m1, int m2, double ks, double kd, double initlen);

	void resizeArray(unsigned int n);

public:
	
	SpringEdgeDataForceField(core::componentmodel::behavior::MechanicalState<DataTypes>* _object);
	
	bool load(const char *filename);
	
	core::componentmodel::behavior::MechanicalState<DataTypes>* getObject() { return object; }

	virtual void init();

	void createDefaultSprings();
	
	virtual void handleEvent( Event* e );
	
	virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);

	virtual void addDForce (VecDeriv& df, const VecDeriv& dx);

	virtual double getPotentialEnergy(const VecCoord& ) { return m_potentialEnergy; }

	Real getStiffness() const {
		return (Real)(m_stiffness.getValue());
	}
	const Real getViscosity() const {
		return (Real)(m_viscosity.getValue());
	}
	const component::topology::EdgeData<Spring> &getSpringArray() const{
		return springArray;
	}
	// -- VisualModel interface
	void draw();
	void initTextures() { }
	void update() { }
	
	// -- Modifiers
	
	void clear(int reserve=0)
	{
		springArray.clear();
		if (reserve) springArray.reserve(reserve);
	}
	/// forward declaration of the loader class used to read spring information from file
	class Loader;
	friend class Loader;
	
};


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
