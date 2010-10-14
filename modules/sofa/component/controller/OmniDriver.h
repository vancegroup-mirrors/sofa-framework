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

//Sensable include
#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <sofa/helper/LCPcalc.h>

#include <sofa/core/behavior/BaseController.h>

namespace sofa
{
	namespace simulation { class Node; }

namespace component
{

namespace controller
{

class ForceFeedback;

using namespace sofa::defaulttype;
using core::objectmodel::Data;

/** Holds data retrieved from HDAPI. */
typedef struct {
		HHD id;
        int nupdates;
        int m_buttonState;					/* Has the device button has been pressed. */
        hduVector3Dd m_devicePosition;                          /* Current device coordinates. */
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
class OmniDriver : public core::behavior::BaseController
{

public:
        Data<double> scale;
        Data<double> forceScale;
        Data<Vec3d> position;
        Data<Vec3d> orientation;
        Data<bool> permanent;

        OmniData	data;

        OmniDriver();
        virtual ~OmniDriver();

		virtual void init();
        void reinit();
        void cleanup();

        void setForceFeedback(ForceFeedback* ff);

private:
        void handleEvent(core::objectmodel::Event *);
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif
