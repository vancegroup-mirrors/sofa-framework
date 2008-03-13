
#include <stdio.h>
#include <assert.h>

#include <GL/glut.h>
#include <GL/gl.h>


#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include "SensAble.h"
#include "Sofa-old/Components/SphereModel.h"
#include "Sofa-old/Components/Sphere.h"
#include "Sofa-old/Components/Common/Vec.h"
#include "Sofa-old/Components/Graph/MechanicalAction.h"
#include "Sofa-old/Components/Graph/UpdateMappingAction.h"

using Sofa::Components::SphereModel;
using Sofa::Components::Sphere;

using Sofa::Components::Common::Vec3d;
using Sofa::Components::Common::Vec3f;

/** Holds data retrieved from HDAPI. */
typedef struct {
	int nupdates;
    HDboolean m_buttonState1;       /* Has the device button has been pressed. */
	HDboolean m_buttonState2;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    HDErrorInfo m_error;
	HDdouble transform[16];
	Quat quat;
} DeviceData;

static DeviceData gServoDeviceData;
static DeviceData deviceData;
static HDSchedulerHandle hUpdateHandle = 0;
static HDSchedulerHandle hCallbackHandle = 0;
static HHD hHD;


HDCallbackCode HDCALLBACK SetForceCallback(void *pUserData);
HDErrorInfo error;
static HDboolean bRenderForce = TRUE;
hduVector3Dd force;

static Vec3d devicePivot;

static HDboolean bRenderPivot = FALSE;

HDCallbackCode HDCALLBACK applyForceCallback(void *pUserData);


/**
 * Checks the state of the gimbal button and gets the position of the device.
 */
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData) {   
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice());
    // Retrieve the current button(s). 
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    // In order to get the specific button 1 state, we use a bitmask to
    // test for the HD_DEVICE_BUTTON_1 bit. 
    gServoDeviceData.m_buttonState1 = (nButtons & HD_DEVICE_BUTTON_1) ? TRUE : FALSE;
	gServoDeviceData.m_buttonState2 = (nButtons & HD_DEVICE_BUTTON_2) ? TRUE : FALSE;
    // Get the current location of the device (HD_GET_CURRENT_POSITION)
    // We declare a vector of three doubles since hdGetDoublev returns 
    // the information in a vector of size 3.
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
	// Get the column major transform...
	hdGetDoublev(HD_CURRENT_TRANSFORM, gServoDeviceData.transform);
    // Also check the error state of HDAPI.
    gServoDeviceData.m_error = hdGetError();
    // Copy the position into our device_data tructure.
    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

/**
 * Copies the state of the gimbal button and the position of the device.
 */
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData) {
    DeviceData *pDeviceData = (DeviceData *) pUserData;
	if (gServoDeviceData.nupdates > 0)
	{
		memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));
		pDeviceData->quat.normalize();
		gServoDeviceData.quat = Quat(0,0,0,0);
		gServoDeviceData.nupdates = 0;
	}
    return HD_CALLBACK_DONE;
}

/**
 * Sets up the device, runs main application loop, cleans up when finished.
 */
int initDevice() {

	deviceData.transform[0*4+0] = 1;
	deviceData.transform[1*4+1] = 1;
	deviceData.transform[2*4+2] = 1;
	deviceData.transform[3*4+3] = 1;
	deviceData.quat[3] = 1;

    HDErrorInfo error;
    // Initialize the device, must be done before attempting to call any hd functions.
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        return -1;
    }
    // Schedule the main scheduler callback that updates the device state.
    //hUpdateHandle = hdScheduleAsynchronous( updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY );

	// Schedule the haptic callback function for generating forces
	// we could do something smart in here, but lets try something stupid instead!
     hCallbackHandle = hdScheduleAsynchronous( applyForceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);

    // Start the servo loop scheduler. 
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");        
        return -1;           
    }
	return 0;
}

/**
 * Performs cleanup, unschedules callbacks and stops the servo loop. 
 */
int uninitDevice() {    
    hdStopScheduler();
    //hdUnschedule(hUpdateHandle);
    hdUnschedule(hCallbackHandle);
    hdDisableDevice(hHD);
    return 0;
}

/**
 * gets the latest data
 */
void getData() {	    
    // Perform a synchronous call to copy the most current device state. 
    hdScheduleSynchronous(copyDeviceDataCallback, &deviceData, HD_MIN_SCHEDULER_PRIORITY);	
}

SensAble::SensAble()
: translation(0, -5, -7), rotation(0,-1,0,1), scale(0.1), tipOffset(-10)
, pivot(1.75, 4.0, -3.75)
{
	rotation.normalize();
	rotation.toMatrix(mrotation);
	std::cout << "Device post transform: scale = "<<scale<<" rotation = "<<rotation<<" translation = "<<translation<<" mrotation = "<<mrotation<<std::endl;
	devicePivot = mrotation.multTranspose(pivot - translation) / scale;
	initDevice();

	sphereModel = new SphereModel();
	double r = 1;
	sphereModel->addSphere(Vec3d(0,0,0),r);
}

SensAble::~SensAble() 
{
	uninitDevice();
}


void SensAble::init()
{
	cout << "init called";
	rigidModel = dynamic_cast<MechanicalModel<RigidTypes>*>(getContext()->getMechanicalModel());
}

void SensAble::updatePosition(double dt) {
	// perhaps update the sphereModel position here...
	Mat3x3d mrot;
	deviceData.quat.toMatrix(mrot);

	double x = deviceData.transform[12] + mrot[0][2]*tipOffset;
	double y = deviceData.transform[13] + mrot[1][2]*tipOffset;
	double z = deviceData.transform[14] + mrot[2][2]*tipOffset;
	Vec3d pos(x,y,z);
	pos *= scale;
	pos = mrotation*pos;
	pos += translation;
	Quat rot;
	//Mat3x3d mrot;
	//for (int i=0; i<3; i++)
	//	for (int j=0; j<3; j++)
	//		mrot[i][j] = deviceData.transform[j*4+i];
	mrot = mrotation * mrot;
	rot.fromMatrix(mrot);

	//sphereModel->setSphere( 0, Vec3d((z-x)*0.5, y, (-x-z)*0.5)*0.5, 0.5 );
	sphereModel->setSphere( 0, pos, 0.5*scale );

	if (deviceData.m_buttonState1)
	{
		Vec3d f = (*sphereModel->getF())[0]/1000;
		//currentForce = Vec3d((-f[2]-f[0])*0.5, f[1], (f[0]-f[2])*0.5);
		currentForce = f;
		if (currentForce.norm()>0.0001)
			cout << currentForce[0] << " " << currentForce[1] << " " << currentForce[2] << "\n";
	}
	else
	{
		currentForce = Vec3d(0,0,0);
	}
	{
		if (!bRenderForce)
		{
			std::cerr << "FORCE ERROR!!!"  << std::endl;
			bRenderForce = TRUE;
		}
		hdScheduleSynchronous(SetForceCallback, &currentForce, HD_DEFAULT_SCHEDULER_PRIORITY);
	}

	(*sphereModel->getF())[0] = Vec3d(0,0,0);

	if (rigidModel != NULL)
	{
		RigidTypes::VecCoord& x = *rigidModel->getX();
		x[0].getCenter() = pos;
		x[0].getOrientation() = rot;
	}

	//hduPrintError(stderr, &error, "status!@");
	static_cast<Sofa::Components::Graph::GNode*>(this->getContext())->execute<Sofa::Components::Graph::MechanicalPropagatePositionAndVelocityAction>();
	static_cast<Sofa::Components::Graph::GNode*>(this->getContext())->execute<Sofa::Components::Graph::UpdateMappingAction>();


}
/*
std::vector<CollisionElement*>& SensAble::getCollisionElements() {
	// perhaps update the sphereModel position here...

	double x = deviceData.transform[13];
	double y = deviceData.transform[14];
	double z = deviceData.transform[15];
	sphereModel->setSphere( 0, x, y, z, 1 );
	
	return sphereModel->getCollisionElements();
}
*/
void SensAble::draw()
{
	getData();
	glPushMatrix();
	Vec3f c;
	if ( deviceData.m_buttonState1 && ! deviceData.m_buttonState2 ) {
		c = Vec3f(1,0,1);
	} else if ( !deviceData.m_buttonState1 && deviceData.m_buttonState2 ) {
		c = Vec3f(0,1,0);
	} else if ( deviceData.m_buttonState1 && deviceData.m_buttonState2 ) {
		c = Vec3f(1,1,0);
	} else {
		c = Vec3f(1,1,1);
	}
	glColor3fv(c.ptr());
	//glMultMatrixd(deviceData.transform);	
	//glutSolidCube(1);
	//glPopMatrix();
	double x = deviceData.transform[12]; // + deviceData.transform[8]*tipOffset;
	double y = deviceData.transform[13]; // + deviceData.transform[9]*tipOffset;
	double z = deviceData.transform[14]; // + deviceData.transform[10]*tipOffset;
	Vec3d p = Vec3d(x, y, z);
	p *= scale;
	p = mrotation*p;
	p += translation;
	//Vec3d p = Vec3d((z-x)*0.5, y, (-x-z)*0.5)*0.5;
	glEnable(GL_LIGHTING);
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, c.ptr());
	glPushMatrix();
	//std::cout << "SPH "<<index<<" "<<x()<<" "<<y()<<" "<<z()<<" "<<radius<<"\n";
	glTranslated(p[0],p[1],p[2]);
	glutSolidSphere(0.49*scale, 16, 8);
	glPopMatrix();
	glDisable(GL_LIGHTING);
	if (currentForce.norm()>0.0001)
	{
		Vec3d f = currentForce; //Vec3d((currentForce[2]-currentForce[0])*0.5,currentForce[1],(-currentForce[0]-currentForce[2])*0.5);
		Vec3d n = f;
		n.normalize();
		glLineWidth(5);
		Vec3d p1,p2;
		p1 = (*sphereModel->getX())[0]+n*0.5*scale;
		p2 = (*sphereModel->getX())[0]+n*0.5*scale+f*2;
		glBegin(GL_LINES);
			glVertex3dv(p1.ptr());
			glVertex3dv(p2.ptr());
		glEnd();
		glLineWidth(1);
	}
	if ( bRenderPivot) {
		Vec3d p0, dir;
		p0 = p;
		dir[0] = deviceData.transform[8];
		dir[1] = deviceData.transform[9];
		dir[2] = deviceData.transform[10];
		dir = mrotation*dir;
		dir.normalize();
		Vec3d dp = p0 - pivot;
		dp -= dir * (dot(dir,dp));
		glLineWidth(5);
		Vec3d p1,p2;
		p1 = pivot;
		p2 = pivot+dp;
		glBegin(GL_LINES);
			glVertex3dv(p1.ptr());
			glVertex3dv(p2.ptr());
		glEnd();
		glLineWidth(1);
	}
}

/**
 * This routine allows the device to provide information about the current 
 * location of the stylus, and contains a mechanism for terminating the 
 * application.  
 * Pressing the button causes the application to display the current location
 * of the device.  
 * Holding the button down for N iterations causes the application to exit. 
 */
/*
void mainLoop(void) {
    static const int kTerminateCount = 1000;
    int buttonHoldCount = 0;

    // Instantiate the structure used to capture data from the device. 
    DeviceData currentData;
    DeviceData prevData;

    // Perform a synchronous call to copy the most current device state. 
    hdScheduleSynchronous(copyDeviceDataCallback, &currentData, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&prevData, &currentData, sizeof(DeviceData));

    printHelp();

    // Run the main loop until the gimbal button is held.
    while (1) {
        // Perform a synchronous call to copy the most current device state.
        // This synchronous scheduler call ensures that the device state
        // is obtained in a thread-safe manner.
        hdScheduleSynchronous(copyDeviceDataCallback, &currentData, HD_MIN_SCHEDULER_PRIORITY);

        // If the user depresses the gimbal button, display the current location information.
        if (currentData.m_buttonState && !prevData.m_buttonState)
        {           
            fprintf(stdout, "Current position: (%g, %g, %g)\n", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2]); 
        }
        else if (currentData.m_buttonState && prevData.m_buttonState)
        {
            // Keep track of how long the user has been pressing the button.
            // If this exceeds N ticks, then terminate the application.
            buttonHoldCount++;

            if (buttonHoldCount > kTerminateCount)
            {
                // Quit, since the user held the button longer than
                // the terminate count.
                break;
            }
        }
        else if (!currentData.m_buttonState && prevData.m_buttonState)
        {
            // Reset the button hold count, since the user stopped holding
            // down the stylus button.
            buttonHoldCount = 0;
        }
        
        // Check if an error occurred.
        if (HD_DEVICE_ERROR(currentData.m_error))
        {
            hduPrintError(stderr, &currentData.m_error, "Device error detected");

            if (hduIsSchedulerError(&currentData.m_error))
            {
                // Quit, since communication with the device was disrupted. 
                fprintf(stderr, "\nPress any key to quit.\n");
                getch();                
                break;
            }
        }

        // Store off the current data for the next loop. 
        memcpy(&prevData, &currentData, sizeof(DeviceData));    
    }
}
*/

/******************************************************************************
 * Main scheduler callback for rendering the anchored spring force.
 *****************************************************************************/
HDCallbackCode HDCALLBACK applyForceCallback(void *pUserData)
{
	static int lastButtons = 0;
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice());
    // Retrieve the current button(s). 
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    // In order to get the specific button 1 state, we use a bitmask to
    // test for the HD_DEVICE_BUTTON_1 bit. 
    gServoDeviceData.m_buttonState1 = (nButtons & HD_DEVICE_BUTTON_1) ? TRUE : FALSE;
	gServoDeviceData.m_buttonState2 = (nButtons & HD_DEVICE_BUTTON_2) ? TRUE : FALSE;
    // Get the current location of the device (HD_GET_CURRENT_POSITION)
    // We declare a vector of three doubles since hdGetDoublev returns 
    // the information in a vector of size 3.
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
	// Get the column major transform...
	hdGetDoublev(HD_CURRENT_TRANSFORM, gServoDeviceData.transform);
    // Also check the error state of HDAPI.
    // Copy the position into our device_data tructure.

	// Swap the X and Z axis
	gServoDeviceData.transform[0]  *= -1;
	gServoDeviceData.transform[1]  *= -1;
	gServoDeviceData.transform[2]  *= -1;
	gServoDeviceData.transform[8]  *= -1;
	gServoDeviceData.transform[9]  *= -1;
	gServoDeviceData.transform[10] *= -1;


	Quat rot;
	Mat3x3d mrot;
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			mrot[i][j] = gServoDeviceData.transform[j*4+i];
	rot.fromMatrix(mrot);
	gServoDeviceData.quat[0] += rot[0];
	gServoDeviceData.quat[1] += rot[1];
	gServoDeviceData.quat[2] += rot[2];
	gServoDeviceData.quat[3] += rot[3];

	if ( (nButtons & HD_DEVICE_BUTTON_2) && !(lastButtons & HD_DEVICE_BUTTON_2) ) {
		bRenderPivot = !bRenderPivot;
	}

	lastButtons = nButtons;

	if ( bRenderForce || bRenderPivot ) {
		hduVector3Dd f(0,0,0);
		if ( bRenderForce) {
	        f += force;
		}
		if ( bRenderPivot ) { //&& (nButtons & HD_DEVICE_BUTTON_1)) {
			Vec3d p0, dir;
			p0[0] = gServoDeviceData.transform[12];
			p0[1] = gServoDeviceData.transform[13];
			p0[2] = gServoDeviceData.transform[14];
			dir[0] = gServoDeviceData.transform[8];
			dir[1] = gServoDeviceData.transform[9];
			dir[2] = gServoDeviceData.transform[10];
			dir.normalize();
			Vec3d dp = p0 - devicePivot;
			dp -= dir * (dot(dir,dp));
			dp *= -0.5;
			if (dp.norm() > 1) dp*= 1/dp.norm();
			f[0] += dp[0];
			f[1] += dp[1];
			f[2] += dp[2];
		}
		hdSetDoublev(HD_CURRENT_FORCE, f);
		// Check if an error occurred while attempting to render the force 
	    gServoDeviceData.m_error = hdGetError();
		if (HD_DEVICE_ERROR(gServoDeviceData.m_error = hdGetError()))
		{
			if (hduIsForceError(&gServoDeviceData.m_error))
			{
				bRenderForce = FALSE;
			}
			else if (hduIsSchedulerError(&gServoDeviceData.m_error))
			{
				return HD_CALLBACK_DONE;
			}
		}
	}
	else
		gServoDeviceData.m_error = hdGetError();

	++gServoDeviceData.nupdates;
	hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

/*
    static hduVector3Dd anchor;
    static HDboolean bRenderForce = FALSE;
    HDErrorInfo error;

    HDint nCurrentButtons, nLastButtons;
    hduVector3Dd position;

    hduVector3Dd force = { 0, 0, 0 };

    hdBeginFrame(hdGetCurrentDevice());

    hdGetDoublev(HD_CURRENT_POSITION, position);    
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButtons);

    if ((nCurrentButtons & HD_DEVICE_BUTTON_1) != 0 &&
        (nLastButtons & HD_DEVICE_BUTTON_1) == 0)
    {
        // Detected button down
        memcpy(anchor, position, sizeof(hduVector3Dd));
        bRenderForce = TRUE;
    }
    else if ((nCurrentButtons & HD_DEVICE_BUTTON_1) == 0 &&
             (nLastButtons & HD_DEVICE_BUTTON_1) != 0)

    {
        // Detected button up 
        bRenderForce = FALSE;

        // Send zero force to the device, or else it will just continue
        // rendering the last force sent
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }

    if (bRenderForce)
    {
        // Compute spring force as F = k * (anchor - pos), which will attract
        // the device position towards the anchor position 
        hduVecSubtract(force, anchor, position);
        hduVecScaleInPlace(force, gSpringStiffness);
                
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }

    hdEndFrame(hdGetCurrentDevice());

    // Check if an error occurred while attempting to render the force 
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        if (hduIsForceError(&error))
        {
            bRenderForce = FALSE;
        }
        else if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}
*/
    
/******************************************************************************
 * Scheduler callback for synchronously changing the rendered force
 *****************************************************************************/
HDCallbackCode HDCALLBACK SetForceCallback(void *pUserData)
{
    Vec3d* pForce = (Vec3d*) pUserData;
	force[0] = (*pForce)[0];
	force[1] = (*pForce)[1];
	force[2] = (*pForce)[2];
    return HD_CALLBACK_DONE;    
}