/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_SPRINGEDGEDATAFORCEFIELD_H

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
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
class SpringEdgeDataForceField : public core::behavior::ForceField<DataTypes>
{
public:
  SOFA_CLASS(SOFA_TEMPLATE(SpringEdgeDataForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

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
	core::behavior::MechanicalState<DataTypes>* object;
	/// the EdgeSet topology used to get the list of edges
	component::topology::EdgeSetTopology<DataTypes> *topology;
	/// the filename where to load the spring information
	DataField<std::string> m_filename;
	/// By default, assume that all edges have the same stiffness
	DataField<double> m_stiffness;
	/// By default, assume that all edges have the same viscosity
	DataField<double> m_viscosity;

	//void addSpringForce(double& potentialEnergy, VecDeriv& f1, const VecCoord& p1, const VecDeriv& v1, VecDeriv& f2, const VecCoord& p2, const VecDeriv& v2, int i, const Spring& spring);
	z	void addSpring(int m1, int m2, SReal ks, SReal kd, SReal initlen);

	void resizeArray(unsigned int n);

public:
	
	SpringEdgeDataForceField(core::behavior::MechanicalState<DataTypes>* _object);
	
	bool load(const char *filename);
	
	core::behavior::MechanicalState<DataTypes>* getObject() { return object; }

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
	void draw();
	
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
