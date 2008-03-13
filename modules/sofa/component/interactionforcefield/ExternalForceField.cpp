#include <sofa/component/interactionforcefield/ExternalForceField.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/core/ObjectFactory.h>
//#include <typeinfo>


namespace sofa
{

namespace component
{

namespace interactionforcefield
{

SOFA_DECL_CLASS(ExternalForceField)

using namespace sofa::defaulttype;

template class ExternalForceField<Vec3dTypes>;
template class ExternalForceField<Vec3fTypes>;
template class ExternalForceField<Vec2dTypes>;
template class ExternalForceField<Vec2fTypes>;
template class ExternalForceField<Vec1dTypes>;
template class ExternalForceField<Vec1fTypes>;
template class ExternalForceField<Vec6dTypes>;
template class ExternalForceField<Vec6fTypes>;
template class ExternalForceField<Rigid3dTypes>;
template class ExternalForceField<Rigid3fTypes>;
template class ExternalForceField<Rigid2dTypes>;
template class ExternalForceField<Rigid2fTypes>;

// Register in the Factory
int ExternalForceFieldClass = core::RegisterObject("Force applied by an external thing")
.add< ExternalForceField<Vec3dTypes> >()
.add< ExternalForceField<Vec3fTypes> >()
.add< ExternalForceField<Vec2dTypes> >()
.add< ExternalForceField<Vec2fTypes> >()
.add< ExternalForceField<Vec1dTypes> >()
.add< ExternalForceField<Vec1fTypes> >()
.add< ExternalForceField<Vec6dTypes> >()
.add< ExternalForceField<Vec6fTypes> >()
.add< ExternalForceField<Rigid3dTypes> >()
.add< ExternalForceField<Rigid3fTypes> >()
.add< ExternalForceField<Rigid2dTypes> >()
.add< ExternalForceField<Rigid2fTypes> >()
;

} // namespace interactionforcefield

} // namespace component

} // namespace sofa

