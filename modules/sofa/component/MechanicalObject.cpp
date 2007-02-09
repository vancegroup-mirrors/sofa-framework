#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/LaparoscopicRigidTypes.h>
#include <sofa/component/MechanicalObject.inl>


namespace sofa
{

namespace component
{

using namespace core::componentmodel::behavior;
using namespace defaulttype;

SOFA_DECL_CLASS(MechanicalObject)

/*
} // namespace component



namespace helper { // \todo Why this must be inside helper namespace

template<class DataTypes>
void create(component::MechanicalObject<DataTypes>*& obj, simulation::tree::xml::ObjectDescription* arg)
{
	obj = new component::MechanicalObject<DataTypes>();
	if (arg->getAttribute("filename"))
	{
		component::MechanicalObjectLoader<DataTypes> loader(obj);
		loader.load(arg->getAttribute("filename"));
		arg->removeAttribute("filename");
	}
	if (obj!=NULL)
	{
		obj->parseFields(arg->getAttributeMap() );
		if (arg->getAttribute("scale")!=NULL) {
			obj->applyScale(atof(arg->getAttribute("scale")));
			arg->removeAttribute("scale");
		}
		if (arg->getAttribute("dx")!=NULL || arg->getAttribute("dy")!=NULL || arg->getAttribute("dz")!=NULL) {
			obj->applyTranslation(atof(arg->getAttribute("dx","0.0")),atof(arg->getAttribute("dy","0.0")),atof(arg->getAttribute("dz","0.0")));
			arg->removeAttribute("dx");
			arg->removeAttribute("dy");
			arg->removeAttribute("dz");
		}
	}
}

} // namespace helper

namespace component
{

Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<Vec3fTypes> > MechanicalObjectVec3fClass("MechanicalObjectVec3f",true);
Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<Vec3dTypes> > MechanicalObjectVec3dClass("MechanicalObjectVec3d",true);
Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<Vec3dTypes> > MechanicalObjectVec3Class("MechanicalObjectVec3",true);
Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<Vec3dTypes> > MechanicalObjectClass("MechanicalObject",true);
Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<RigidTypes> > MechanicalObjectRigidClass("MechanicalObjectRigid",true);
Creator< simulation::tree::xml::ObjectFactory, MechanicalObject<LaparoscopicRigidTypes> > MechanicalObjectLaparoscopicRigidClass("LaparoscopicObject",true);
*/

int MechanicalObjectVec3fClass = core::RegisterObject("mechanical state vectors")
        .add< MechanicalObject<Vec3dTypes> >()
        .add< MechanicalObject<Vec3fTypes> >()
        .add< MechanicalObject<RigidTypes> >()
        .add< MechanicalObject<LaparoscopicRigidTypes> >()
        ;

// template specialization must be in the same namespace as original namespace for GCC 4.1

template <>
void MechanicalObject<defaulttype::Vec3dTypes>::getIndicesInSpace(std::vector<unsigned>& indices,Real xmin,Real xmax,Real ymin,Real ymax,Real zmin,Real zmax) const
{
    const VecCoord& x = *getX();
    for( unsigned i=0; i<x.size(); ++i )
    {
        if( x[i][0] >= xmin && x[i][0] <= xmax && x[i][1] >= ymin && x[i][1] <= ymax && x[i][2] >= zmin && x[i][2] <= zmax )
        {
            indices.push_back(i);
        }
    }
}
template <>
void MechanicalObject<defaulttype::Vec3fTypes>::getIndicesInSpace(std::vector<unsigned>& indices,Real xmin,Real xmax,Real ymin,Real ymax,Real zmin,Real zmax) const
{
    const VecCoord& x = *getX();
    for( unsigned i=0; i<x.size(); ++i )
    {
        if( x[i][0] >= xmin && x[i][0] <= xmax && x[i][1] >= ymin && x[i][1] <= ymax && x[i][2] >= zmin && x[i][2] <= zmax )
        {
            indices.push_back(i);
        }
    }
}

// overload for rigid bodies: use the center
template<>
void MechanicalObject<defaulttype::RigidTypes>::getIndicesInSpace(std::vector<unsigned>& indices,Real xmin,Real xmax,Real ymin,Real ymax,Real zmin,Real zmax) const
{
    const VecCoord& x = *getX();
    for( unsigned i=0; i<x.size(); ++i )
    {
        if( x[i].getCenter()[0] >= xmin && x[i].getCenter()[0] <= xmax && x[i].getCenter()[1] >= ymin && x[i].getCenter()[1] <= ymax && x[i].getCenter()[2] >= zmin && x[i].getCenter()[2] <= zmax )
        {
            indices.push_back(i);
        }
    }
}

// g++ 4.1 requires template instantiations to be declared on a parent namespace from the template class.

template class MechanicalObject<defaulttype::Vec3fTypes>;
template class MechanicalObject<defaulttype::Vec3dTypes>;
template class MechanicalObject<defaulttype::RigidTypes>;
template class MechanicalObject<defaulttype::LaparoscopicRigidTypes>;

} // namespace component

} // namespace sofa
