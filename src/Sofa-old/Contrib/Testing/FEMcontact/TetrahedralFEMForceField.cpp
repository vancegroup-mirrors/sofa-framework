#include "TetrahedralFEMForceField.inl"
#include "Common/Vec3Types.h"
#include "XML/DynamicNode.h"
#include "Sofa-old/Core/MechanicalObject.h"
#include "XML/ForceFieldNode.h"

//#include <typeinfo>

namespace Sofa
{

namespace Components
{

SOFA_DECL_CLASS(TetrahedralFEMForceField)

using namespace Common;

template class TetrahedralFEMForceField<Vec3dTypes>;
template class TetrahedralFEMForceField<Vec3fTypes>;

template<class DataTypes>
void create(TetrahedralFEMForceField<DataTypes>*& obj, XML::Node<Core::ForceField>* arg)
{
	XML::createWithParent< TetrahedralFEMForceField<DataTypes>, Core::MechanicalObject<DataTypes> >(obj, arg);
	if (obj!=NULL)
	{
		obj->setPoissonRatio((typename TetrahedralFEMForceField<DataTypes>::Real)atof(arg->getAttribute("poissonRatio","0.49")));
		obj->setYoungModulus((typename TetrahedralFEMForceField<DataTypes>::Real)atof(arg->getAttribute("youngModulus","100000")));
		std::string method = arg->getAttribute("method","");
		if (method == "small")
			obj->setMethod(TetrahedralFEMForceField<DataTypes>::SMALL);
		else if (method == "large")
			obj->setMethod(TetrahedralFEMForceField<DataTypes>::LARGE);
		else if (method == "polar")
			obj->setMethod(TetrahedralFEMForceField<DataTypes>::POLAR);
		obj->setUpdateStiffnessMatrix(std::string(arg->getAttribute("updateStiffnessMatrix","false"))=="true");
		obj->setComputeGlobalMatrix(std::string(arg->getAttribute("computeGlobalMatrix","false"))=="true");
	}
}

Creator< XML::ForceFieldNode::Factory, TetrahedralFEMForceField<Vec3dTypes> > TetrahedronFEMForceFieldVec3dClass("TetrahedralFEMForceField", true);
Creator< XML::ForceFieldNode::Factory, TetrahedralFEMForceField<Vec3fTypes> > TetrahedronFEMForceFieldVec3fClass("TetrahedralFEMForceField", true);

} // namespace Components

} // namespace Sofa
