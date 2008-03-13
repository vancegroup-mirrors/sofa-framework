/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the Free   *
* Software Foundation; either version 2 of the License, or (at your option)    *
* any later version.                                                           *
*                                                                              *
* This program is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for     *
* more details.                                                                *
*                                                                              *
* You should have received a copy of the GNU General Public License along with *
* this program; if not, write to the Free Software Foundation, Inc., 51        *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                    *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef __MenuCallbacks_h__
#define __MenuCallbacks_h__

#include <FL/Fl.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_File_Browser.H>
#include <FL/fl_ask.H>
#include <iostream>
#include <sofa/simulation/automatescheduler/Automate.h>


namespace sofa
{

namespace gui
{

namespace fltk
{

void	IdleCB(void*);
void	LoadFileCB(void);
void	SaveFileCB(void);
void	SaveFileAsCB(void);
void	QuitCB();
//void	ComputeDeformationCB();
void	ChangeForceIntensityCB(double value);
void	SmoothOnOffCB(bool value);
void	CollisionOnOffCB(bool value);
void	VisualOnOffCB(bool value);
void	BehaviorOnOffCB(bool value);
void	MappingOnOffCB(bool value);
void	AddThreadCB(void);
void	displayAutomateCB(void *);
//void	displayFPSCB(int);
void DtCB(double value);
void    eventNewStep();

class FLTKDrawCB : public sofa::simulation::automatescheduler::Automate::DrawCB
{
public:
	void drawFromAutomate();
	static FLTKDrawCB instance;
};

} // namespace fltk

} // namespace gui

} // namespace sofa

#endif	/* __MenuCallbacks_h__ */
