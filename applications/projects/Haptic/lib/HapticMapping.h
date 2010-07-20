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
#ifndef SOFA_COMPONENT_MAPPING_HAPTICMAPPING_H
#define SOFA_COMPONENT_MAPPING_HAPTICMAPPING_H


#include "ChaiDevice.h"
#include "sofa/core/behavior/MechanicalMapping.h"
#include "sofa/component/mapping/RigidRigidMapping.h"


namespace sofa{

	namespace component{

		namespace mapping{

                  using sofa::core::objectmodel::ChaiDevice;
                  using sofa::component::mapping::RigidRigidMapping;
                  using sofa::core::behavior::MechanicalMapping;
                  using sofa::core::behavior::MechanicalState;

/**
*	\brief specialized rigid to rigid mapping which link 
*	an haptic device and a rigid mechanical object
*
*	It's a special extension of a RigidRigidMapping, where the IN object
*	isn't used, and is dynamicaly replace by the haptic device datas
*
*	You need to specialized it with the type of your rigid mechanical object
*	that represent your device in SOFA
*
*/
template <class DeviceType>
class HapticMapping 
: 
public RigidRigidMapping< MechanicalMapping< MechanicalState<DeviceType>, MechanicalState<DeviceType> > >{


private :
	
	/// The haptic device
	ChaiDevice* chaiDevice;

	/// Catch the OpenGlInitializedEvent to activate the mapping
	/// (GL_MODELVIEW_MATRIX must be initialize to apply the mapping)
	///
	/// see apply method for more detail about this feature 
	void handleEvent(core::objectmodel::Event *);

	/// Indicate if openGl is initialized (GL_MODELVIEW_MATRIX must be initialize to apply the mapping)
	///
	/// see apply method for more detail about this feature 
	Data< bool > openGlInitialized;

public :
	typedef MechanicalState<DeviceType> Out;

	/// Constructor taking the device DOF (the Output model) as parameter
	HapticMapping(MechanicalState<DeviceType>* device);

	/// Destructor
	///
	/// Delete the device in order to stop the haptic loop
	~HapticMapping();
	
	/// Apply the mapping on position vectors. overwrited from Mapping 
	///
	/// We don't use the second parameter (the input model) wich is replace by the physical device
	/// When OpenGl is initialized, this function convert the device position in the sofa world position,
	/// in order that your device always move in the same axes, in front of the user
	void apply( typename DeviceType::VecCoord& device, const typename DeviceType::VecCoord& /* notUsed */);


	/// Apply the reverse mapping on force vectors. overwrited from Mapping
	///
	/// Does nothing yet
	void applyJT( typename DeviceType::VecDeriv& /*device*/, const typename DeviceType::VecDeriv& /*notUsed*/) {};
	
	/// Apply the mapping on derived (velocity, displacement) vectors. overwrited from Mapping
	///
	/// Does nothing yet
	void applyJ( typename DeviceType::VecDeriv& /*device*/, const typename DeviceType::VecDeriv& /*notUsed*/) {};
	
	
	// **********************************************************
	//  The two method following (canCreate & create) can't be removed 
	//  from .h file because of the template of the output
	// **********************************************************


	/// Pre-construction check method called by ObjectFactory. Overtwrited from Mapping.
	///
	/// This implementation read the object2 attributes and check
   	/// if it is compatible with the output model types of this
   	/// mapping. We jump over the test of the input model 
	template<class T>
	static bool canCreate(	T*& obj, 
				core::objectmodel::BaseContext* /* context */,
				core::objectmodel::BaseObjectDescription* arg)
	{
       		if (arg->findObject(arg->getAttribute("object2","..")) == NULL)
           		std::cerr << "Cannot create "<<className(obj)<<" as object2 is missing.\n";
        	if (dynamic_cast<Out*>(arg->findObject(arg->getAttribute("object2",".."))) == NULL)
            		return false;
        	return true;
	}


	/// Construction method called by ObjectFactory. Overtwrited from Mapping.
    	///
    	/// This implementation read the object2 attributes to
    	/// find the output models of this mapping.
	template<class T>
	static void create(	T*& obj, 
				core::objectmodel::BaseContext* context, 
				core::objectmodel::BaseObjectDescription* arg)
	{
		obj = new T((arg?dynamic_cast<Out*>(arg->findObject(arg->getAttribute("object2",".."))):NULL));
		if (context) context->addObject(obj);
		if ((arg) && (arg->getAttribute("object2")))
		{
			obj->object2.setValue( arg->getAttribute("object2") );
			arg->removeAttribute("object2");
		}
		if (arg) obj->parse(arg);
	}
	
	






}; // class HapticMapping


		} // namespace mapping

	} // namespace component

} // namespace sofa

#endif
