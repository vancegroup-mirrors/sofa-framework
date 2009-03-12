/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
/*
*   ChaiDevice
*
*   Abstraction of an Haptic Device interfaced through CHAI3D library
*   It initialize the device, launch the haptic loop in a thread
*   and provide function to get Device information in sofa primitives
*   ( position for example )
*
*   version 0.1
*
*   // TO FIX : Can't instantiate twice the device, the chai initialisation crash
*/
#ifndef SOFA_CORE_OBJECTMODEL_CHAIDEVICE_H
#define SOFA_CORE_OBJECTMODEL_CHAIDEVICE_H


#include <sofa/defaulttype/VecTypes.h>
#include <Chai3D/CMeta3dofPointer.h>

using sofa::defaulttype::Vector3;


namespace sofa{
	namespace core{
		namespace objectmodel{



/**
* Abstraction of an haptic device controlled
* by the chai3D library.
*/
class ChaiDevice
{
public:
	ChaiDevice();
	~ChaiDevice();
	
	Vector3 getCoord();
public:
	// a 3D cursor which represents the haptic device
	cMeta3dofPointer* device;
};



} // namespace objectmodel
} // namespace core
} // namespace sofa


#endif
