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
*   author Gurvan Le Moigno
*
*   // TO FIX : Can't instantiate twice the device, the chai initialisation crash
*/
#include "ChaiDevice.h"


// include needed by Chai
#include "Chai3D/CWorld.h"
#include "Chai3D/CPrecisionTimer.h"
#include "Chai3D/CVector3d.h"
#include "Chai3D/CMeta3dofPointer.h"



namespace sofa{
namespace core{
namespace objectmodel{



// haptic timer callback for the thread loop
cPrecisionTimer timer;



/**
 *  Really simple haptic loop which update the position of the device
 * 
 */
void hapticsLoop(void *a_pUserData)
{	
	cMeta3dofPointer* device = static_cast<cMeta3dofPointer*>(a_pUserData);
	device->updatePose();	
};


/**
 * Constructor
 * Initialise the device in a chai3D world and launch the haptic loop
 */
ChaiDevice::ChaiDevice(){


	cWorld* world = new cWorld();
	
	// Try to get a device
	// see chai3D documentation
	this->device = new cMeta3dofPointer(world, 0);
	
	// try to initialize device
	int isInitialize = device->initialize();
	if( isInitialize == -1){
		std::cerr << "Can't initialize Haptic Device" << std::endl;
	}
	
	world->addChild(device);
	
	// Fit the best with our SOFA world
	// TODO : perhaps have to calculate/ get this information from SOFA
	device->setWorkspace(10,10,10);

	// open communication to the device
	device->start();
	
	// start haptic timer callback
	// 0 means maximum rate as possible
	timer.set(0, hapticsLoop, device);
};


/**
 * Destructor
 * stop the haptic loop
 */
ChaiDevice::~ChaiDevice(){
	timer.stop();
	device->stop();
	delete device;
};


/**
* Link between SOFA and CHAI3D
* It convert the device position in the chai3D world
* into a SOFA Vector3 primitive
*
*/
Vector3 ChaiDevice::getCoord(){
	cVector3d pos = device->m_deviceGlobalPos;
	// not the same axes between chai and sofa !!
	// x -> z
	// y -> x
	// z -> y
	return Vector3(pos.y,pos.z,pos.x);
};


} // namespace objectmodel
} // namespace core
} // namespace sofa
