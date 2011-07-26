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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/objectmodel/Context.h>
// #include <sofa/simulation/common/Visitor.h>


namespace sofa
{

namespace core
{

namespace objectmodel
{

Context::Context()
  : is_activated(initData(&is_activated, true, "activated", "To Activate a node"))
  , worldGravity_(initData(&worldGravity_, Vec3((SReal)0,(SReal)-9.81,(SReal)0),"gravity","Gravity in the world coordinate system"))
  , dt_(initData(&dt_,0.01,"dt","Time step"))
  , time_(initData(&time_,0.,"time","Current time"))
  , animate_(initData(&animate_,false,"animate","Animate the Simulation(applied at initialization only)"))
  , showVisualModels_           (initData(&showVisualModels_,           -1, "showVisualModels","display Visual Models"))
  , showBehaviorModels_         (initData(&showBehaviorModels_,         -1,"showBehaviorModels","display Behavior Models"))
  , showCollisionModels_        (initData(&showCollisionModels_,        -1,"showCollisionModels","display Collision Models"))
  , showBoundingCollisionModels_(initData(&showBoundingCollisionModels_,-1,"showBoundingCollisionModels","display Bounding Collision Models"))
  , showMappings_               (initData(&showMappings_,               -1,"showMappings","display Mappings"))
  , showMechanicalMappings_     (initData(&showMechanicalMappings_,     -1,"showMechanicalMappings","display Mechanical Mappings"))
  , showForceFields_            (initData(&showForceFields_,            -1,"showForceFields","display Force Fields"))
  , showInteractionForceFields_ (initData(&showInteractionForceFields_, -1,"showInteractionForceFields","display Interaction Force Fields"))
  , showWireFrame_              (initData(&showWireFrame_,              -1,"showWireFrame","display in WireFrame"))
  , showNormals_                (initData(&showNormals_,                -1,"showNormals","display Normals"))
#ifdef SOFA_SMP
  , showProcessorColor_                (initData(&showProcessorColor_,                -1,"showProcessorColor","display Processor Color"))
#endif
#ifdef SOFA_SMP
  ,  processor(initData(&processor,(int )-1,"processor","assigned processor"))
  ,  gpuPrioritary(initData(&gpuPrioritary,false,"gpuPrioritary","node should be executed on GPU")),
//  is_partition_(initData(&is_partition_,false,"partition","is a parallel partition"))
  partition_(0)
#endif
{

#ifdef SOFA_SMP
    is_partition_.setValue(false);
#endif
    addAlias(&showVisualModels_,           "showAll"); addAlias(&showVisualModels_,           "showVisual");

    addAlias(&showBehaviorModels_,         "showAll"); addAlias(&showBehaviorModels_,         "showBehavior");
    addAlias(&showForceFields_,            "showAll"); addAlias(&showForceFields_,            "showBehavior");
    addAlias(&showInteractionForceFields_, "showAll"); addAlias(&showInteractionForceFields_, "showBehavior");


    addAlias(&showCollisionModels_,        "showAll"); addAlias(&showCollisionModels_,        "showCollision");
    addAlias(&showBoundingCollisionModels_,"showAll"); addAlias(&showBoundingCollisionModels_,"showCollision");

    addAlias(&showMappings_,               "showAll"); addAlias(&showMappings_,               "showMapping");
    addAlias(&showMechanicalMappings_,     "showAll"); addAlias(&showMechanicalMappings_,     "showMapping");

    //setDt(objectmodel::BaseContext::getDt());
    //setTime(objectmodel::BaseContext::getTime());
    //setAnimate(objectmodel::BaseContext::getAnimate());
    //setShowCollisionModels(objectmodel::BaseContext::getShowCollisionModels());
    //setShowBoundingCollisionModels(objectmodel::BaseContext::getShowBoundingCollisionModels());
    //setShowBehaviorModels(objectmodel::BaseContext::getShowBehaviorModels());
    //setShowVisualModels(objectmodel::BaseContext::getShowVisualModels());
    //setShowMappings(objectmodel::BaseContext::getShowMappings());
    //setShowMechanicalMappings(objectmodel::BaseContext::getShowMechanicalMappings());
    //setShowForceFields(objectmodel::BaseContext::getShowForceFields());
    //setShowInteractionForceFields(objectmodel::BaseContext::getShowInteractionForceFields());
    //setShowWireFrame(objectmodel::BaseContext::getShowWireFrame());
    //setShowNormals(objectmodel::BaseContext::getShowNormals());
}

/// The Context is active
bool Context::isActive() const {return is_activated.getValue();}

/// State of the context
void Context::setActive(bool val)
{
	is_activated.setValue(val);
}



/// Simulation timestep
double Context::getDt() const
{
    return dt_.getValue();
}

/// Simulation time
double Context::getTime() const
{
    return time_.getValue();
}


/// Gravity vector in world coordinates
const Context::Vec3& Context::getGravity() const
{
    return worldGravity_.getValue();
}



/// Animation flag
bool Context::getAnimate() const
{
    return animate_.getValue();
}

/// Display flags: Collision Models
bool Context::getShowCollisionModels() const
{
  if (showCollisionModels_.getValue() < 0) return false;
  else return showCollisionModels_.getValue() != 0;
}

/// Display flags: Bounding Collision Models
bool Context::getShowBoundingCollisionModels() const
{
  if (showBoundingCollisionModels_.getValue() < 0) return false;
  else return showBoundingCollisionModels_.getValue()!= 0;
}

/// Display flags: Behavior Models
bool Context::getShowBehaviorModels() const
{
  if (showBehaviorModels_.getValue() < 0) return false;
  else  return showBehaviorModels_.getValue()!= 0;
}

/// Display flags: Visual Models
bool Context::getShowVisualModels() const
{
  if (showVisualModels_.getValue() < 0) return true;
  else return showVisualModels_.getValue()!= 0;
}

/// Display flags: Mappings
bool Context::getShowMappings() const
{
  if (showMappings_.getValue() < 0) return false;
  else return showMappings_.getValue()!= 0;
}

/// Display flags: Mechanical Mappings
bool Context::getShowMechanicalMappings() const
{
  if (showMechanicalMappings_.getValue() < 0) return false;
  else return showMechanicalMappings_.getValue()!= 0;
}

/// Display flags: ForceFields
bool Context::getShowForceFields() const
{
  if (showForceFields_.getValue() < 0) return false;
  else return showForceFields_.getValue()!= 0;
}

/// Display flags: InteractionForceFields
bool Context::getShowInteractionForceFields() const
{
  if (showInteractionForceFields_.getValue() < 0) return false;
  else return showInteractionForceFields_.getValue()!= 0;
}

/// Display flags: WireFrame
bool Context::getShowWireFrame() const
{
  if (showWireFrame_.getValue() < 0) return false;
  else return showWireFrame_.getValue()!= 0;
}

/// Display flags: Normal
bool Context::getShowNormals() const
{
  if (showNormals_.getValue() < 0) return false;
  else return showNormals_.getValue()!= 0;
}
#ifdef SOFA_SMP
/// Display flags: Normal
bool Context::getShowProcessorColor() const
{
  if (showProcessorColor_.getValue() < 0) return false;
  else return showProcessorColor_.getValue()!= 0;
}
#endif


//===============================================================================

/// Simulation timestep
void Context::setDt(double val)
{
    dt_.setValue(val);
}

/// Simulation time
void Context::setTime(double val)
{
    time_.setValue(val);
}

/// Gravity vector
// void Context::setGravity(const Vec3& g)
// {
// 	gravity_ = g;
// }

/// Gravity vector
void Context::setGravity(const Vec3& g)
{
    worldGravity_ .setValue(g);
}

/// Animation flag
void Context::setAnimate(bool val)
{
    animate_.setValue(val);
}

/// Display flags: Collision Models
void Context::setShowCollisionModels(bool val)
{
    showCollisionModels_.setValue(val);
}

/// Display flags: Bounding Collision Models
void Context::setShowBoundingCollisionModels(bool val)
{
    showBoundingCollisionModels_.setValue(val);
}

/// Display flags: Behavior Models
void Context::setShowBehaviorModels(bool val)
{
    showBehaviorModels_.setValue(val);
}

/// Display flags: Visual Models
void Context::setShowVisualModels(bool val)
{
    showVisualModels_.setValue(val);
}

/// Display flags: Mappings
void Context::setShowMappings(bool val)
{
    showMappings_.setValue(val);
}

/// Display flags: Mechanical Mappings
void Context::setShowMechanicalMappings(bool val)
{
    showMechanicalMappings_.setValue(val);
}

/// Display flags: ForceFields
void Context::setShowForceFields(bool val)
{
    showForceFields_.setValue(val);
}

/// Display flags: InteractionForceFields
void Context::setShowInteractionForceFields(bool val)
{
    showInteractionForceFields_.setValue(val);
}

/// Display flags: WireFrame
void Context::setShowWireFrame(bool val)
{
    showWireFrame_.setValue(val);
}

/// Display flags: Normals
void Context::setShowNormals(bool val)
{
    showNormals_.setValue(val);
}

#ifdef SOFA_SMP
void Context::setShowProcessorColor(bool val)
{
    showProcessorColor_.setValue(val);
}
#endif


//======================


void Context::copyContext(const Context& c)
{
    // BUGFIX 12/01/06 (Jeremie A.): Can't use operator= on the class as it will copy other data in the BaseContext class (such as name)...
    // *this = c;

    copySimulationContext(c);
    copyVisualContext(c);
}
#ifdef SOFA_SMP
int Context::getProcessor() const
{
    return processor.getValue();
}
void Context::setProcessor(int p) 
{
    processor.setValue(p);
}
#endif


void Context::copySimulationContext(const Context& c)
{
  worldGravity_.setValue(c.getGravity());  ///< Gravity IN THE WORLD COORDINATE SYSTEM.
  setDt(c.getDt());
  setTime(c.getTime());
  setAnimate(c.getAnimate());
#ifdef SOFA_SMP
if(c.gpuPrioritary.getValue())
  gpuPrioritary.setValue(true);
#endif



#ifdef SOFA_SMP
if(!partition_){
    if(processor.getValue()!=-1)
      is_partition_.setValue(true);
    if(is_partition()){
    
      partition_= new Iterative::IterativePartition();
//          partition_->setCPU(processor.getValue());
    }
}
   if(processor.getValue()==-1&&c.processor.getValue()!=-1){
  	processor.setValue(c.processor.getValue());
         is_partition_.setValue(true);
      }
   if(c.is_partition()&&!partition_){
     partition_=c.getPartition();
     is_partition_.setValue(true);
    }
    if((gpuPrioritary.getValue())&&partition_)
       {
	        partition_->setGPUPrioritary();
       }

#endif

}

void Context::copyVisualContext(const Context& c)
{
  setShowCollisionModels(c.getShowCollisionModels());
  setShowBoundingCollisionModels(c.getShowBoundingCollisionModels());
  setShowBehaviorModels(c.getShowBehaviorModels());
  setShowVisualModels(c.getShowVisualModels());
  setShowMappings(c.getShowMappings());
  setShowMechanicalMappings(c.getShowMechanicalMappings());
  setShowForceFields(c.getShowForceFields());
  setShowInteractionForceFields(c.getShowInteractionForceFields());
  setShowWireFrame(c.getShowWireFrame());
  setShowNormals(c.getShowNormals());
#ifdef SOFA_SMP
  setShowProcessorColor(c.getShowProcessorColor());
#endif
}


void Context::fusionVisualContext(const Context& c)
{
  setShowCollisionModels(getShowCollisionModels() || c.getShowCollisionModels());
  setShowBoundingCollisionModels(getShowBoundingCollisionModels() || c.getShowBoundingCollisionModels());
  setShowBehaviorModels(getShowBehaviorModels() || c.getShowBehaviorModels());
  setShowVisualModels(getShowVisualModels() || c.getShowVisualModels());
  setShowMappings(getShowMappings() || c.getShowMappings());
  setShowMechanicalMappings(getShowMechanicalMappings() || c.getShowMechanicalMappings());
  setShowForceFields(getShowForceFields() || c.getShowForceFields());
  setShowInteractionForceFields(getShowInteractionForceFields() || c.getShowInteractionForceFields());
  setShowWireFrame(getShowWireFrame() || c.getShowWireFrame());
  setShowNormals(getShowNormals() || c.getShowNormals());
}




} // namespace objectmodel

} // namespace core

} // namespace sofa

