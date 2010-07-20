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
#ifndef SOFA_COMPONENT_MAPPING_HAPTICMAPPING_INL
#define SOFA_COMPONENT_MAPPING_HAPTICMAPPING_INL

#include "HapticMapping.h"
#include <sofa/defaulttype/RigidTypes.h>
#include "sofa/core/objectmodel/GLInitializedEvent.h"
#include "sofa/simulation/common/AnimateBeginEvent.h"
#include "sofa/component/mapping/RigidRigidMapping.h"




namespace sofa{
	namespace component{
		namespace mapping{



using sofa::core::behavior::MechanicalMapping;
using sofa::component::mapping::RigidRigidMapping;

using sofa::defaulttype::Matrix4;
using sofa::defaulttype::Vector3;
using sofa::defaulttype::Quat;

typedef sofa::defaulttype::RigidCoord<3, double> coord;
typedef sofa::defaulttype::Vec4f Vector4f;

template<class DeviceType>
HapticMapping<DeviceType>::HapticMapping(MechanicalState<DeviceType>* device)
:
// we initialize the RigidRigidMapping with our device in output and input, but the input is never used
// It's only initialize because if input is NULL, a lot of function doesn't accept it.  
RigidRigidMapping< MechanicalMapping< MechanicalState<DeviceType>, MechanicalState<DeviceType> > >(device, device)
{
	// Initialisation of the openGlInitialized boolean. Inside the constructor because of bugs with template...
	// False by default because the simulation start before the animation
	openGlInitialized = this->initData( &openGlInitialized, false, "openGlInitialized", "Tell HapticMapping that OpenGl is initialized or not");
	this->chaiDevice = new ChaiDevice();
	// in order to catch the OpenGlInitializeEvent
	this->f_listening = true;
};





template<class DeviceType>
HapticMapping<DeviceType>::~HapticMapping()
{
	// delete explicitly the device to stop the haptic loop
	delete this->chaiDevice;
};


// Convert a vector3 in the device world into a vector3 in the SOFA world
// Simuate that the device move in a fixed axes in front of the camera,
// and that the device doesn't move with the scene when its rotate/translate
Vector3 convertFromCameraToSceneCoord(Vector3 in){
		
		// get the camera matrix
		float camMat[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, (float *) &camMat);
		
		// transform the matrix in a Sofa Matrix
		Matrix4 camera_axes(camMat);
		
		// transform the vector in a four value one
		Vector4f pos(in,1);
		
		// translation of the camera in the camera axes
		Vector4f trans(-camera_axes[3][0],-camera_axes[3][1],0, 0);
		
		// Real position of the device in the camera axes
		Vector4f trans_nr = pos + trans;
		
		// Rotation of the position of the device in order to
		// find position in the scene axes
		Vector4f newPos_4 = camera_axes * trans_nr;
		
		// Transform the Vector4 in Vector3
		return Vector3(newPos_4.x(),newPos_4.y(),newPos_4.z());
};


template<class DeviceType>
void HapticMapping<DeviceType>::apply(	typename DeviceType::VecCoord& device,
					const typename DeviceType::VecCoord& /* notUsed */ )
{
	// Test if visualisation is started otherwise it crash because GL_MODELVIEW_MATRIX isn't initialize
	if(openGlInitialized.getValue()){
		device[0] = coord(convertFromCameraToSceneCoord(chaiDevice->getCoord()),Quat::identity());
	}
};


template<class DeviceType>
void HapticMapping<DeviceType>::handleEvent(core::objectmodel::Event *event){
	// only catch the GLInitializedEvent
	if (dynamic_cast<sofa::core::objectmodel::GLInitializedEvent *>(event))
	{
		openGlInitialized.setValue(true); // Ok, run !
	}
};




		} // namespace mapping

	} // namespace component

} // namespace sofa




#endif
