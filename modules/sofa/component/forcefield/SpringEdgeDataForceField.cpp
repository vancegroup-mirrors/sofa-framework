// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/forcefield/SpringEdgeDataForceField.inl>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/simulation/tree/xml/ObjectFactory.h>
//#include <typeinfo>


namespace sofa
{

namespace component
{

namespace forcefield
{

SOFA_DECL_CLASS(SpringEdgeDataForceField)

using namespace sofa::defaulttype;

template class SpringEdgeDataForceField<Vec3dTypes>;
template class SpringEdgeDataForceField<Vec3fTypes>;

template<class DataTypes>
void create(SpringEdgeDataForceField<DataTypes>*& obj, simulation::tree::xml::ObjectDescription* arg)
{
	simulation::tree::xml::createWithParent< SpringEdgeDataForceField<DataTypes>, core::componentmodel::behavior::MechanicalState<DataTypes> >(obj, arg);
	if (obj != NULL)
	{
	    obj->parseFields( arg->getAttributeMap() );
		//if (arg->getAttribute("filename"))
		//	obj->load(arg->getAttribute("filename"));
	}
}

Creator<simulation::tree::xml::ObjectFactory, SpringEdgeDataForceField<Vec3dTypes> > SpringEdgeDataForceFieldVec3dClass("SpringEdgeDataForceField", true);
Creator<simulation::tree::xml::ObjectFactory, SpringEdgeDataForceField<Vec3fTypes> > SpringEdgeDataForceFieldVec3fClass("SpringEdgeDataForceField", true);

} // namespace forcefield

} // namespace component

} // namespace sofa

