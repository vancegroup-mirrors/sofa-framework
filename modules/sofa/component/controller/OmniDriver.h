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
#ifndef SOFA_COMPONENT_ODESOLVER_OMNISOLVER_H
#define SOFA_COMPONENT_ODESOLVER_OMNISOLVER_H

#include <sofa/core/componentmodel/behavior/OdeSolver.h>
#include <sofa/component/odesolver/OdeSolverImpl.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/component/linearsolver/NewMatMatrix.h>
#include <sofa/component/linearsolver/NewMatVector.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/component/container/ArticulatedHierarchyContainer.h>

#include <sofa/component/controller/ForceFeedback.h>

#include <sofa/core/componentmodel/behavior/BaseController.h>
//Sensable include
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <sofa/helper/LCPcalc.h>

namespace sofa
{

namespace component
{

namespace controller
{


using namespace sofa::defaulttype;
using namespace sofa::component::linearsolver;
using core::objectmodel::Data;

/** Holds data retrieved from HDAPI. */
typedef struct {
        int nupdates;
        HDboolean m_buttonState1;       /* Has the device button has been pressed. */
        HDboolean m_buttonState2;       /* Has the device button has been pressed. */
        hduVector3Dd m_devicePosition; /* Current device coordinates. */
        HDErrorInfo m_error;
        Vec3d pos;
        Quat quat;
        bool ready;
        bool stop;
} DeviceData;

typedef struct {
        ForceFeedback* forceFeedback;
        simulation::Node *context;
        Mat3x3d rotation;
        Vec3d translation;
        double scale;
        double forceScale;
        DeviceData servoDeviceData;
        DeviceData deviceData;

        bool permanent_feedback;
} OmniData;

/**
* Omni driver
*/
class OmniDriver : public core::componentmodel::behavior::BaseController
{

public:
        Data<double> scale;
        Data<double> forceScale;
        Data<Vec3d> position;
        Data<Vec3d> orientation;
        Data<bool> permanent;

        simulation::Node *context; //->propagateEvent()
        //ForceFeedback* forceFeedback;
        OmniData* data;
        void init();
        void reinit();
        void setForceFeedback(ForceFeedback* ff);
        OmniDriver();
        ~OmniDriver();
        void cleanup();
private:
        void handleEvent(core::objectmodel::Event *);
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif
