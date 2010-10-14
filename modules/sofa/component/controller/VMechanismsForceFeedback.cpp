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
#include <sofa/component/controller/VMechanismsForceFeedback.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/mastersolver/MasterContactSolver.h>
#include <sofa/component/mastersolver/MasterConstraintSolver.h>
#include <sofa/helper/LCPcalc.h>

using namespace std;
using namespace sofa::defaulttype;

namespace sofa
{
namespace component
{
namespace controller
{

int VMechanismsForceFeedbackClass = sofa::core::RegisterObject("Virtual Mechanisms LCP force feedback for haptic device")
#ifndef SOFA_FLOAT
    .add< VMechanismsForceFeedback<sofa::defaulttype::Vec1dTypes> >()
    .add< VMechanismsForceFeedback<sofa::defaulttype::Rigid3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    //.add< VMechanismsForceFeedback<sofa::defaulttype::Vec1fTypes> >()
    //.add< VMechanismsForceFeedback<sofa::defaulttype::Rigid3fTypes> >()
#endif
    ;

#ifndef SOFA_FLOAT
template class SOFA_COMPONENT_CONTROLLER_API VMechanismsForceFeedback<Vec1dTypes>;
template class SOFA_COMPONENT_CONTROLLER_API VMechanismsForceFeedback<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
//template class SOFA_COMPONENT_CONTROLLER_API VMechanismsForceFeedback<Vec1fTypes>;
//template class SOFA_COMPONENT_CONTROLLER_API VMechanismsForceFeedback<Rigid3fTypes>;
#endif

SOFA_DECL_CLASS(VMechanismsForceFeedback)

} // namespace controller
} // namespace component
} // namespace sofa
