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
#include <sofa/component/controller/OmniDriver.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/ObjectFactory.h>
#include <math.h>
#include <iostream>

//Required by for vfork
#include <sys/types.h>
// #include <unistd.h>

//event handling
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/OmniEvent.h>

//force feedback
#include <sofa/component/controller/NullForceFeedback.h>
#include <sofa/component/controller/EnslavementForceFeedback.h>

#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>





//sensable namespace
using namespace sofa::defaulttype;
using namespace sofa::simulation::tree;


/*
static DeviceData gServoDeviceData;
static DeviceData deviceData;
static HHD hHD;
bool isInitialized = false;
static HDSchedulerHandle hForceHandle = 0;
static HDSchedulerHandle hStateHandle = 0;
static hduVector3Dd force;
*/

double prevTime;

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;
using namespace core::componentmodel::behavior;

//static DeviceData gServoDeviceData;
//static DeviceData deviceData;
//static DeviceData previousData;
static HHD hHD = HD_INVALID_HANDLE ;
static bool isInitialized = false;
static HDSchedulerHandle hStateHandle = HD_INVALID_HANDLE;


HDCallbackCode HDCALLBACK stateCallback(void *userData)
{
	OmniData* data = static_cast<OmniData*>(userData);
	//FIXME : Apparenlty, this callback is run before the mechanical state initialisation. I've found no way to know whether the mechcanical state is initialized or not, so i wait ...
	//static int wait = 0;

	if (data->servoDeviceData.stop)
	{
		return HD_CALLBACK_DONE;
	}

	if (!data->servoDeviceData.ready)
	{
		return HD_CALLBACK_CONTINUE;
	}

	HHD hapticHD = hdGetCurrentDevice();
	hdBeginFrame(hapticHD);

	//static int renderForce = true;
	int nButtons = 0;

	// Retrieve the current button(s).
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);

	data->servoDeviceData.m_buttonState1 = (nButtons & HD_DEVICE_BUTTON_1) ? true : false;
	data->servoDeviceData.m_buttonState2 = (nButtons & ~HD_DEVICE_BUTTON_1) ? true : false;

	//hdGetDoublev(HD_CURRENT_POSITION, data->servoDeviceData.m_devicePosition);
	// Get the column major transform
	HDdouble transform[16];
	hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

	// Swap the X and Z axis

	transform[0]  *= 1;//previously was 1
	transform[1]  *= 1;//previously was 1
	transform[2]  *= 1;//previously was 1

//	transform[4]  *= -1;
//	transform[5]  *= -1;
//	transform[6]  *= -1;

	transform[8]  *= 1;//previously was 1
	transform[9]  *= 1;//previously was 1
	transform[10] *= 1;//previously was 1

/*
	data->servoDeviceData.transform[12+0]  *= 1;
	data->servoDeviceData.transform[12+1]  *= -1;
	data->servoDeviceData.transform[12+2]  *= -1;
*/
	Mat3x3d mrot;

	Quat rot;
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			mrot[i][j] = transform[j*4+i];

	mrot = data->rotation * mrot;
	data->servoDeviceData.pos = data->translation + data->rotation * (Vec3d(transform[12+0], transform[12+1], transform[12+2]) * data->scale);

	rot.fromMatrix(mrot);
	rot.normalize();

	if ((rot[0]*data->servoDeviceData.quat[0]+rot[1]*data->servoDeviceData.quat[1]+rot[2]*data->servoDeviceData.quat[2]+rot[3]*data->servoDeviceData.quat[3]) < 0)
		for (int i=0;i<4;i++)
			rot[i] *= -1;

	data->servoDeviceData.quat[0] = rot[0];
	data->servoDeviceData.quat[1] = rot[1];
	data->servoDeviceData.quat[2] = rot[2];
	data->servoDeviceData.quat[3] = rot[3];

	//if(data->servoDeviceData.m_buttonState1)
	//	renderForce = !renderForce;

	double fx=0.0, fy=0.0, fz=0.0;
	if (data->forceFeedback != NULL)
		(data->forceFeedback)->computeForce(data->servoDeviceData.pos[0], data->servoDeviceData.pos[1], data->servoDeviceData.pos[2], data->servoDeviceData.quat[0], data->servoDeviceData.quat[1], data->servoDeviceData.quat[2], data->servoDeviceData.quat[3], fx, fy, fz);


	double currentForce[3];
	currentForce[0] = ( data->rotation[0][0] * fx + data->rotation[1][0] * fy + data->rotation[2][0] * fz) * data->forceScale;
	currentForce[1] = ( data->rotation[0][1] * fx + data->rotation[1][1] * fy + data->rotation[2][1] * fz) * data->forceScale;
	currentForce[2] = ( data->rotation[0][2] * fx + data->rotation[1][2] * fy + data->rotation[2][2] * fz) * data->forceScale;

 	if(data->servoDeviceData.m_buttonState1 || data->permanent_feedback)
		hdSetDoublev(HD_CURRENT_FORCE, currentForce);

	++data->servoDeviceData.nupdates;
	hdEndFrame(hapticHD);


	 HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error during scheduler callback");
		if (hduIsSchedulerError(&error))
		{
		return HD_CALLBACK_DONE;
		}
	}
/*
 	OmniX = data->servoDeviceData.transform[12+0]*0.1;
	OmniY =	data->servoDeviceData.transform[12+1]*0.1;
	OmniZ =	data->servoDeviceData.transform[12+2]*0.1;
*/

	return HD_CALLBACK_CONTINUE;
}

void exitHandler()
{
    hdStopScheduler();
    hdUnschedule(hStateHandle);
/*
    if (hHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(hHD);
        hHD = HD_INVALID_HANDLE;
    }
*/
}


HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
	OmniData *data = static_cast<OmniData*>(pUserData);
	memcpy(&data->deviceData, &data->servoDeviceData, sizeof(DeviceData));
	//data->servoDeviceData.quat[0] = data->servoDeviceData.quat[1] = data->servoDeviceData.quat[2] = data->servoDeviceData.quat[3] = 0.0;
	data->servoDeviceData.nupdates = 0;
	data->servoDeviceData.ready = true;
    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK stopCallback(void *pUserData)
{
	OmniData *data = static_cast<OmniData*>(pUserData);
	data->servoDeviceData.stop = true;
    return HD_CALLBACK_DONE;
}

/**
 * Sets up the device,
 */
int initDevice(OmniData* data)
{
	if (isInitialized) return 0;
	isInitialized = true;
	//deviceData.transform[0*4+0] = 1;
	//deviceData.transform[1*4+1] = 1;
	//deviceData.transform[2*4+2] = 1;
	//deviceData.transform[3*4+3] = 1;
	data->deviceData.quat[0] = 0;
	data->deviceData.quat[1] = 0;
	data->deviceData.quat[2] = 0;
	data->deviceData.quat[3] = 1;
	data->servoDeviceData.quat[0] = 0;
	data->servoDeviceData.quat[1] = 0;
	data->servoDeviceData.quat[2] = 0;
	data->servoDeviceData.quat[3] = 1;
    HDErrorInfo error;
    // Initialize the device, must be done before attempting to call any hd functions.
	if (hHD == HD_INVALID_HANDLE)
{
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize the device");
		return -1;
    }
    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));

	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_MAX_FORCE_CLAMPING);

    // Start the servo loop scheduler.
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        return -1;
    }
}

	data->servoDeviceData.ready = false;
	data->servoDeviceData.stop = false;
    hStateHandle = hdScheduleAsynchronous( stateCallback, data, HD_MAX_SCHEDULER_PRIORITY);

    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

	return 0;
}



OmniDriver::OmniDriver()
: scale(initData(&scale, 0.1, "scale","Default scale applied to the Phantom Coordinates. "))
, forceScale(initData(&forceScale, 1.0, "forceScale","Default forceScale applied to the force feedback. "))
, position(initData(&position, Vec3d(0,0,0), "position","Default position applied to the Phantom Coordinates"))
, orientation(initData(&orientation, Vec3d(0,0,0), "orientation","Default orientation applied to the Phantom Coordinates"))
, permanent(initData(&permanent, false, "permanent" , "Apply the force feedback permanently"))
{
	serr<<"toto"<<sendl;
	this->f_listening.setValue(true);
	data = new OmniData;
	data->forceFeedback = new NullForceFeedback();
}

OmniDriver::~OmniDriver()
{
}

void OmniDriver::cleanup()
{
	sout << "OmniDriver::cleanup()" << sendl;
	hdScheduleSynchronous(stopCallback, data, HD_MIN_SCHEDULER_PRIORITY);
    //exitHandler();
    isInitialized = false;
//    delete forceFeedback;
}

void OmniDriver::setForceFeedback(ForceFeedback* ff)
{
	sout << "change ff" << endl;
	if(data->forceFeedback)
		delete data->forceFeedback;
	data->forceFeedback = ff;
};

void OmniDriver::init()
{
	sout << "OmniDriver::init()" << sendl;
	context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
	double fx, fy, fz;
	data->forceFeedback->computeForce(0,0,0, 0,0,0,1, fx, fy, fz);
	//OmniData* data = new OmniData();
	data->context = context;
	//data->forceFeedback = &forceFeedback;
	data->scale = scale.getValue();
	data->forceScale = forceScale.getValue();
	data->translation = position.getValue();
	Vector3 radV = orientation.getValue() * M_PI/180;
	Quaternion q = helper::Quater<double>::createFromRotationVector(radV);
	q.toMatrix(data->rotation);
	initDevice(data);
	sout << "OmniDriver::init() done" << sendl;

	data->permanent_feedback = permanent.getValue();
}

void OmniDriver::reinit()
{
	init();
}

void OmniDriver::handleEvent(core::objectmodel::Event *event)
{
	if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
	{
		//getData(); // copy data->servoDeviceData to gDeviceData
    hdScheduleSynchronous(copyDeviceDataCallback, data, HD_MIN_SCHEDULER_PRIORITY);
if (data->deviceData.ready)
{
    data->deviceData.quat.normalize();
		//sout << "driver is working ! " << data->servoDeviceData.transform[12+0] << endl;
		sofa::core::objectmodel::OmniEvent::State omniState = sofa::core::objectmodel::OmniEvent::Button1;
//		sofa::core::objectmodel::OmniEvent omniEvent(omniState, deviceData.transform[12+0]*0.1,
//		deviceData.transform[12+1]*0.1,
//		deviceData.transform[12+2]*0.1);
//sout << "omni pos="<<data->deviceData.pos<<" quat="<<data->deviceData.quat<<sendl;


		sofa::core::objectmodel::OmniEvent omniEvent(omniState, data->deviceData.pos, data->deviceData.quat,  data->deviceData.m_buttonState1);

		this->getContext()->propagateEvent(&omniEvent);
}
	}
}

int OmniDriverClass = core::RegisterObject("Solver to test compliance computation for new articulated system objects")
    .add< OmniDriver >();

SOFA_DECL_CLASS(OmniDriver)


} // namespace controller

} // namespace component

} // namespace sofa
