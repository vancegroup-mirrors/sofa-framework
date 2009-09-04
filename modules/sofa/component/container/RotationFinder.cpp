/*
 * RotationFinder.cpp
 *
 *  Created on: 14 avr. 2009
 *      Author: froy
 */
#define SOFA_COMPONENT_CONTAINER_ROTATIONFINDER_CPP
#include <sofa/component/container/RotationFinder.inl>
#include <sofa/core/ObjectFactory.h>
namespace sofa
{

namespace component
{

namespace container
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(RotationFinder)

// Register in the Factory
int RotationFinderClass = core::RegisterObject("RotationFinder")
#ifndef SOFA_FLOAT
.add< RotationFinder<Vec3dTypes> >()
//.add< RotationFinder<Vec2dTypes> >()
#endif
#ifndef SOFA_DOUBLE
.add< RotationFinder<Vec3fTypes> >()
//.add< RotationFinder<Vec2fTypes> >()
#endif
;
#ifndef SOFA_FLOAT
template class SOFA_COMPONENT_CONTAINER_API RotationFinder<defaulttype::Vec3dTypes>;
//template class RotationFinder<defaulttype::Vec2dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_COMPONENT_CONTAINER_API RotationFinder<defaulttype::Vec3fTypes>;
//template class RotationFinder<defaulttype::Vec2fTypes>;
#endif

} // namespace constraint

} // namespace component

} // namespace sofa
