/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include "ContactCorrection.inl"
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace component
{
namespace collision
{
	using namespace sofa::defaulttype;

SOFA_DECL_CLASS(ContactCorrection)

int ContactCorrectionClass = core::RegisterObject("")
.add< ContactCorrection<Vec1dTypes> >()
.add< ContactCorrection<Rigid3dTypes> >()
;

	//template class ContactCorrection<Vec3dTypes>;
    //template class ContactCorrection<Vec3fTypes>;
    //template class ContactCorrection<Vec2dTypes>;
    //template class ContactCorrection<Vec2fTypes>;
    template class ContactCorrection<Vec1dTypes>;
    //template class ContactCorrection<Vec1fTypes>;
	template class ContactCorrection<Rigid3dTypes>;
    //template class ContactCorrection<Rigid3fTypes>;
    //template class ContactCorrection<Rigid2dTypes>;
    //template class ContactCorrection<Rigid2fTypes>;


} // namespace collision

} // namespace component

} // namespace sofa
