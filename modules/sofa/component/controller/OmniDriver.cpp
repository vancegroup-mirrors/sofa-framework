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

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/OmniEvent.h>
//
////force feedback
#include <sofa/component/controller/ForceFeedback.h>
#include <sofa/component/controller/NullForceFeedback.h>
//
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
//
#include <sofa/simulation/common/Node.h>
#include <cstring>
#include <sofa/helper/SimpleTimer.h>


#ifdef WIN32
// BUGFIX(Jeremie A. 02-05-2009): put OpenHaptics libs here instead of the project file to work around a bug in qmake when there is a space in an environment variable used to look-up a library
#pragma comment(lib,"hl.lib")
#pragma comment(lib,"hd.lib")
#pragma comment(lib,"hdu.lib")
#endif

//sensable namespace
using namespace sofa::defaulttype;

double prevTime;

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;
using namespace core::behavior;

//static DeviceData gServoDeviceData;
//static DeviceData deviceData;
//static DeviceData previousData;
static HHD hHD = HD_INVALID_HANDLE ;
static bool isInitialized = false;
static HDSchedulerHandle hStateHandle = HD_INVALID_HANDLE;

void printError_OLD(FILE *stream, const HDErrorInfo *error,
                   const char *message)
{
    fprintf(stream, "%s\n", hdGetErrorString(error->errorCode));
    fprintf(stream, "HHD: %X\n", error->hHD);
    fprintf(stream, "Error Code: %X\n", error->errorCode);
    fprintf(stream, "Internal Error Code: %d\n", error->internalErrorCode);
    fprintf(stream, "Message: %s\n", message);
}

bool isSchedulerError_OLD(const HDErrorInfo *error)
{
    switch (error->errorCode)
    {
        case HD_COMM_ERROR:
        case HD_COMM_CONFIG_ERROR:
        case HD_TIMER_ERROR:
        case HD_INVALID_PRIORITY:
        case HD_SCHEDULER_FULL:
            return true;

        default:
            return false;
    }
}

static bool firstCB_OLD = true;

sofa::helper::TSimpleTimer<1000,10> mytimer;

HDCallbackCode HDCALLBACK updateDeviceCallback_OLD(void *userData)
{
        cout << "[OldOmni] UpdateCallBack" << endl;
        /*if (!firstCB_OLD)
            mytimer.stop();
        firstCB_OLD = false;
        mytimer.start("CB");*/

	OmniData* data = static_cast<OmniData*>(userData);
        //FIXME : Apparently, this callback is run before the mechanical state initialisation. I've found no way to know whether the mechcanical state is initialized or not, so i wait ...
        //static int wait = 0;

        /*if (data->servoDeviceData.stop)
	{
                cout << "HD_DONE" << endl;
		return HD_CALLBACK_DONE;
	}

	if (!data->servoDeviceData.ready)
	{
                cout << "HD_CONTINUE" << endl;
		return HD_CALLBACK_CONTINUE;
        }
        cout << "PASSED" << endl;*/

	HHD hapticHD = hdGetCurrentDevice();
	hdBeginFrame(hapticHD);

	data->servoDeviceData.id = hapticHD;

	//static int renderForce = true;

	// Retrieve the current button(s).
	hdGetIntegerv(HD_CURRENT_BUTTONS, &data->servoDeviceData.m_buttonState);

        hdGetDoublev(HD_CURRENT_POSITION, data->servoDeviceData.m_devicePosition);

        //cout << "Getting the position of device: " << data->servoDeviceData.m_devicePosition << endl;

        // Get the column major transform
        HDdouble transform[16];
	hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

	// Swap the X and Z axis

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

        /*mytimer.step("Force");*/

	double fx=0.0, fy=0.0, fz=0.0;
	if (data->forceFeedback != NULL)
		(data->forceFeedback)->computeForce(data->servoDeviceData.pos[0], data->servoDeviceData.pos[1], data->servoDeviceData.pos[2], data->servoDeviceData.quat[0], data->servoDeviceData.quat[1], data->servoDeviceData.quat[2], data->servoDeviceData.quat[3], fx, fy, fz);

        mytimer.step("OpenHaptics");

	double currentForce[3];
	currentForce[0] = ( data->rotation[0][0] * fx + data->rotation[1][0] * fy + data->rotation[2][0] * fz) * data->forceScale;
	currentForce[1] = ( data->rotation[0][1] * fx + data->rotation[1][1] * fy + data->rotation[2][1] * fz) * data->forceScale;
	currentForce[2] = ( data->rotation[0][2] * fx + data->rotation[1][2] * fy + data->rotation[2][2] * fz) * data->forceScale;

        //cout << "Current force" << currentForce << endl;
        //if((data->servoDeviceData.m_buttonState & HD_DEVICE_BUTTON_1) || data->permanent_feedback)
                hdSetDoublev(HD_CURRENT_FORCE, currentForce);

        /*++data->servoDeviceData.nupdates;*/



        HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
                printError_OLD(stderr, &error, "Error during scheduler callback");
                if (isSchedulerError_OLD(&error))
		{
			return HD_CALLBACK_DONE;
		}
        }
        hdEndFrame(hapticHD);
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


HDCallbackCode HDCALLBACK copyDeviceDataCallback_OLD(void *pUserData)
{
	OmniData *data = static_cast<OmniData*>(pUserData);
	memcpy(&data->deviceData, &data->servoDeviceData, sizeof(DeviceData));
	//data->servoDeviceData.quat[0] = data->servoDeviceData.quat[1] = data->servoDeviceData.quat[2] = data->servoDeviceData.quat[3] = 0.0;        
        cout << "Copy CB:" << endl << data->servoDeviceData.m_devicePosition << endl << data->servoDeviceData.pos <<  endl << data->servoDeviceData.quat << endl;

	data->servoDeviceData.nupdates = 0;
	data->servoDeviceData.ready = true;
        return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK stopCallback_OLD(void *pUserData)
{
	OmniData *data = static_cast<OmniData*>(pUserData);
	data->servoDeviceData.stop = true;
    return HD_CALLBACK_DONE;
}

/**
 * Sets up the device,
 */
int initDevice_OLD(OmniData& data)
{
        cout << "[OldOmni] Init Device Start" << endl;
	if (isInitialized) return 0;
	isInitialized = true;

	data.deviceData.quat[0] = 0;
	data.deviceData.quat[1] = 0;
	data.deviceData.quat[2] = 0;
	data.deviceData.quat[3] = 1;

	data.servoDeviceData.quat[0] = 0;
	data.servoDeviceData.quat[1] = 0;
	data.servoDeviceData.quat[2] = 0;
	data.servoDeviceData.quat[3] = 1;

        HDErrorInfo error;
        // Initialize the device, must be done before attempting to call any hd functions.
	if (hHD == HD_INVALID_HANDLE)
	{
		hHD = hdInitDevice(HD_DEFAULT_DEVICE);
		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
                        //printError_OLD(stderr, &error, "Failed to initialize the device");
                        hduPrintError(stderr, &error, "Failed to initialize the device");
			return -1;
		}
		printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));

                hStateHandle = hdScheduleAsynchronous( updateDeviceCallback_OLD, (void*) &data, HD_MAX_SCHEDULER_PRIORITY);
                //hStateHandle = hdScheduleAsynchronous( updateDeviceCallback_OLD, 0, HD_MAX_SCHEDULER_PRIORITY);

                hdEnable(HD_FORCE_OUTPUT);
                hdEnable(HD_MAX_FORCE_CLAMPING);

		// Start the servo loop scheduler.
		hdStartScheduler();
		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
                        //printError_OLD(stderr, &error, "Failed to start the scheduler");
                        hduPrintError(stderr, &error, "Failed to start the scheduler");
			return -1;
		}
	}

	data.servoDeviceData.ready = false;
	data.servoDeviceData.stop = false;


    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        printError_OLD(stderr, &error, "Failed to initialize haptic device");
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
        cerr<<"toto"<<sendl;
	this->f_listening.setValue(true);
	data.forceFeedback = new NullForceFeedback();
}

OmniDriver::~OmniDriver()
{
}

void OmniDriver::cleanup()
{
        cout << "OmniDriver::cleanup()" << sendl;
        hdScheduleSynchronous(stopCallback_OLD, (void*) &data, HD_MIN_SCHEDULER_PRIORITY);
    //exitHandler();
    isInitialized = false;
//    delete forceFeedback;
}

void OmniDriver::setForceFeedback(ForceFeedback* ff)
{
        cout << "change ff" << endl;
	if(data.forceFeedback)
		delete data.forceFeedback;
	data.forceFeedback = ff;
};

void OmniDriver::init()
{
        cout << "OmniDriver::init()" << sendl;
	this->reinit();
}

void OmniDriver::reinit()
{
	simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
	ForceFeedback *ff = context->getTreeObject<ForceFeedback>();
	
	if(ff)
	{
		this->setForceFeedback(ff);
	}

	data.scale = scale.getValue();
	data.forceScale = forceScale.getValue();
	data.translation = position.getValue();
	Vector3 radV = orientation.getValue() * M_PI/180;
	Quaternion q = helper::Quater<double>::createFromRotationVector(radV);
	q.toMatrix(data.rotation);
        initDevice_OLD(data);
        cout << "OmniDriver::init() done" << sendl;

	data.permanent_feedback = permanent.getValue();
}

void OmniDriver::handleEvent(core::objectmodel::Event *event)
{
	if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
	{
		//getData(); // copy data->servoDeviceData to gDeviceData
                hdScheduleSynchronous(copyDeviceDataCallback_OLD, (void *) &data, HD_MIN_SCHEDULER_PRIORITY);
		if (data.deviceData.ready)
		{
			data.deviceData.quat.normalize();                        

			sofa::core::objectmodel::OmniEvent omniEvent(data.deviceData.id, data.deviceData.pos, data.deviceData.quat, data.deviceData.m_buttonState);

                        cout << "driver is working ! " << data.deviceData.pos << endl;

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
