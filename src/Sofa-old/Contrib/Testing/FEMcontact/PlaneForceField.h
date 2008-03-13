#ifndef SOFA_COMPONENTS_PLANEFORCEFIELD_H
#define SOFA_COMPONENTS_PLANEFORCEFIELD_H

#include "Sofa-old/Core/ForceField.h"
#include "Sofa-old/Core/MechanicalModel.h"
#include "Sofa-old/Abstract/VisualModel.h"
#include "Sofa-old/Contrib/Testing/FEMcontact/TetrahedralFEMForceField.h"

namespace Sofa
{

namespace Components
{

template<class DataTypes>
class PlaneForceField : public Core::ForceField<DataTypes>, public Abstract::VisualModel
{
public:
	typedef Core::ForceField<DataTypes> Inherit;
	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename Coord::value_type Real;
	
protected:
	Core::MechanicalModel<DataTypes>* object;
	TetrahedralFEMForceField<DataTypes>* fem;
	
	Deriv planeNormal;
	Real planeD;
	
	Real stiffness;
	
	std::vector<unsigned int> contacts;

	VecDeriv _force;
	
public:
	PlaneForceField(Core::MechanicalModel<DataTypes>* object, const std::string& /*name*/="")
	: object(object), planeD(0), stiffness(500), fem(0)
	{
	}
	
	PlaneForceField(Core::MechanicalModel<DataTypes>* object, TetrahedralFEMForceField<DataTypes>* fem)
	: object(object), fem(fem), planeD(0), stiffness(500)
	{
	}
	
	void setPlane(const Deriv& normal, Real d)
	{
		planeNormal = normal;
		planeD = d;
	}
	
	void setStiffness(Real stiff)
	{
		stiffness = stiff;
	}
	
	virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);
	
	virtual void addDForce (VecDeriv& df, const VecDeriv& dx);

        virtual double getPotentialEnergy(const VecCoord& x);
	
	// -- VisualModel interface
	void draw();
	void initTextures() { }
	void update() { }
};

} // namespace Components

} // namespace Sofa

#endif
