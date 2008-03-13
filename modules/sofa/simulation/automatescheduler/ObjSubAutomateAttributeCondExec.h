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
#ifndef SOFA_SIMULATION_AUTOMATESCHEDULER_OBJSUBAUTOMATEATTRIBUTECONDEXEC_H
#define SOFA_SIMULATION_AUTOMATESCHEDULER_OBJSUBAUTOMATEATTRIBUTECONDEXEC_H

namespace sofa
{

namespace simulation
{

namespace automatescheduler
{

class Cond;

/////////////////////////////////////
// ObjSubAutomateAttributeCondExec //
/////////////////////////////////////

template <class ObjSubAutomateAttributeCondClass>
class ObjSubAutomateAttributeCondExec : public Cond
{
	ObjSubAutomateAttributeCondClass attribute;
	std::vector<ObjSubAutomateAttributeCondClass *> execList;

	public:

		ObjSubAutomateAttributeCondExec(std::vector<ObjSubAutomateAttributeCondClass *> );
		void addAttribute(ObjSubAutomateAttributeCondClass _attribute);
		bool eval(void);
		std::vector<ObjSubAutomateAttributeCondClass *> *getExecList(void);
};

} // namespace automatescheduler

} // namespace simulation

} // namespace sofa

/// \todo Only include inline methods if necessary
#include "ObjSubAutomateAttributeCondExec.inl"

#endif
