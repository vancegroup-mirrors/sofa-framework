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
#include <stdio.h>
#include <assert.h>

#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>


#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include "SensAble.h"

#include <sofa/simulation/tree/GNode.h> 
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/component/typedef/Sofa_typedef.h>

#ifdef WIN32
// BUGFIX(Jeremie A. 02-05-2009): put OpenHaptics libs here instead of the project file to work around a bug in qmake when there is a space in an environment variable used to look-up a library
#pragma comment(lib,"hl.lib")
#pragma comment(lib,"hd.lib")
#pragma comment(lib,"hdu.lib")
#endif

using namespace sofa::defaulttype;
using namespace sofa::simulation::tree;
using namespace sofa::simulation;

SOFA_DECL_CLASS(SensAble)

int SensAbleClass = sofa::core::RegisterObject("Phantom device")
.add< SensAble >();

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
static volatile HDboolean bRenderForce = false;
static hduVector3Dd force;
static volatile HDboolean bRenderPivot = false;
static Vec3d devicePivot;


HDCallbackCode HDCALLBACK applyForceCallback(void *pUserData);


/**
 * Checks the state of the gimbal button and gets the position of the device.
 */
HDCallbackCode HDCALLBACK updateDeviceCallback(void * /* pUserData */) {   
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice());
    // Retrieve the current button(s). 
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    // In order to get the specific button 1 state, we use a bitmask to
    // test for the HD_DEVICE_BUTTON_1 bit. 
    gServoDeviceData.m_buttonState1 = (nButtons & HD_DEVICE_BUTTON_1) ? true : false;
	gServoDeviceData.m_buttonState2 = (nButtons & HD_DEVICE_BUTTON_2) ? true : false;
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

bool isInitialized = false;

/**
 * Sets up the device, runs main application loop, cleans up when finished.
 */
int initDevice() {
	if (isInitialized) return 0;
	isInitialized = true;
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
: scale(initData(&scale, 0.1, "scale","Default scale applied to the Phantom Coordinates. "))
, tipOffset(initData(&tipOffset, -10.0, "tipOffset","tipOffset. [needs information]"))
, xform(initData(&xform, sofa::defaulttype::RigidTypes::Coord(), "xform","Transformation applied to the Phantom coordinates."))
, xform2(initData(&xform2, sofa::defaulttype::RigidTypes::Coord(), "xform2","Additional rotation."))
, pivot(initData(&pivot, sofa::defaulttype::Vec3d(1.75, 4.0, -3.75), "pivot","Constrained point where the tool is pivoting."))
, flip(initData(&flip, true, "flip","Rotate the instrument by 180�"))
  , bDraw(initData(&bDraw, true, "draw","Enable drawing of Sensable force state"))
, key1(initData(&key1, '0', "key1","Key event generated when the first button is pressed")), button1State(false)
, key2(initData(&key2, '1', "key2","Key event generated when the second button is pressed")), button2State(false)
{
    this->f_listening.setValue(true);

	sofa::defaulttype::Quat _rotation;
	_rotation = xform.getValue().getOrientation();

	_rotation.normalize();
	_rotation.toMatrix(mrotation);
	std::cout << "Device post transform: scale = " << scale.getValue() <<" rotation = "<<xform.getValue().getOrientation()<<" translation = "<<xform.getValue().getCenter()<<" mrotation = "<<mrotation<<std::endl;
	devicePivot = mrotation.multTranspose(pivot.getValue() - xform.getValue().getCenter()) / scale.getValue();
	initDevice();

	GNode* sphnode = new GNode;
	sphereModel = new sofa::component::collision::SphereModel();
	sphnode->addObject(sphereModel);
    MechanicalObject3* DOF = new MechanicalObject3;
	sphnode->addObject(DOF);
	sphereModel->init();
	DOF->init();
	sphereModel->addSphere(Vec3d(0,0,0), 1);
}

void SensAble::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        switch(ev->getKey())
        {
	case '7':
	case 'f':
	    bRenderForce = !bRenderForce;
	    std::cout << "HAPTICS FORCE "<<(bRenderForce?"ENABLED":"DISABLED")<<std::endl;
            break;
	case '8':
	case 'p':
	    bRenderPivot = !bRenderPivot;
	    std::cout << "HAPTICS PIVOT "<<(bRenderPivot?"ENABLED":"DISABLED")<<std::endl;
            break;
        }
    }
}

SensAble::~SensAble() 
{
	//uninitDevice();
}


void SensAble::init()
{
	cout << "init called";
	rigidModel = dynamic_cast<sofa::component::container::MechanicalObject<RigidTypes>*>(getContext()->getMechanicalState());
	reinit();
}


void SensAble::reinit()
{
	sofa::defaulttype::Quat _rotation;
	_rotation = xform.getValue().getOrientation();
	sofa::defaulttype::Quat _rotation2;
	_rotation2 = xform2.getValue().getOrientation();

	_rotation.normalize();
	_rotation2.normalize();
	if (_rotation2[3] != 1)
	{
	    _rotation = _rotation2 * _rotation;
	    xform.beginEdit()->getOrientation() = _rotation;
	    xform.endEdit();
	    xform2.beginEdit()->getOrientation() = sofa::defaulttype::Quat(0,0,0,1);
	    xform2.endEdit();
	}
	_rotation.toMatrix(mrotation);
	std::cout << "Device post transform: scale = " << scale.getValue() <<" rotation = "<<xform.getValue().getOrientation()<<" translation = "<<xform.getValue().getCenter()<<" mrotation = "<<mrotation<<std::endl;
	devicePivot = mrotation.multTranspose(pivot.getValue() - xform.getValue().getCenter()) / scale.getValue();
}

void SensAble::updatePosition(double /* dt */) {
	// perhaps update the sphereModel position here...
	Mat3x3d mrot;
	deviceData.quat.toMatrix(mrot);

	double x = deviceData.transform[12] + mrot[0][2]*tipOffset.getValue();
	double y = deviceData.transform[13] + mrot[1][2]*tipOffset.getValue();
	double z = deviceData.transform[14] + mrot[2][2]*tipOffset.getValue();
	Vec3d pos(x,y,z);
	pos *= scale.getValue();
	pos = mrotation*pos;
	pos += xform.getValue().getCenter();
	Quat rot;
	//Mat3x3d mrot;
	//for (int i=0; i<3; i++)
	//	for (int j=0; j<3; j++)
	//		mrot[i][j] = deviceData.transform[j*4+i];
	mrot = mrotation * mrot;
	//std::cout << "mrot="<<mrot<<std::endl;
	if (flip.getValue())
	{
	//sofa::defaulttype::Quat r2(0,1,0,0);
	Mat3x3d mrot2;
	mrot2[0][0]=1;
	mrot2[1][1]=-1;
	mrot2[2][2]=-1;
	//r2.toMatrix(mrot2);
	mrot = mrot * mrot2;
	}
	rot.fromMatrix(mrot);
	Mat3x3d finalrot;
	rot.toMatrix(finalrot);
	//std::cout << "frot="<<finalrot<<std::endl;

	//sphereModel->setSphere( 0, Vec3d((z-x)*0.5, y, (-x-z)*0.5)*0.5, 0.5 );
	sphereModel->setSphere( 0, pos, 0.5*scale.getValue() );

	
	typedef sofa::core::behavior::BaseMechanicalState::VecId VecId;

	//if (deviceData.m_buttonState1)
	if (bRenderForce)
	{

	    Vec3d f = (*sphereModel->getMechanicalState()->getF())[0];
	if (rigidModel != NULL)
	{
	    MechanicalComputeForceVisitor(VecId::force()).execute( rigidModel->getContext() );
	    f += (*rigidModel->getF())[0].getVCenter();
	}
		//currentForce = Vec3d((-f[2]-f[0])*0.5, f[1], (f[0]-f[2])*0.5);
		currentForce = f * 1.0e-10;
		//if (currentForce.norm()>0.0001)
			cout << currentForce[0] << " " << currentForce[1] << " " << currentForce[2] << "\n";
	}
	else
	{
		currentForce = Vec3d(0,0,0);
	}
	{
	    //if (!bRenderForce)
	    //{
	    //	std::cerr << "FORCE ERROR!!!"  << std::endl;
	    //	bRenderForce = true;
	    //}
	    if (bRenderForce)
		hdScheduleSynchronous(SetForceCallback, &currentForce, HD_DEFAULT_SCHEDULER_PRIORITY);
	}

	(*sphereModel->getMechanicalState()->getF())[0] = Vec3d(0,0,0);
	if (rigidModel != NULL)
	{
	    MechanicalResetForceVisitor(VecId::force()).execute( rigidModel->getContext() );
	}

	if (rigidModel != NULL)
	{
		RigidTypes::VecCoord& x = *rigidModel->getX();
		x[0].getCenter() = pos;
		x[0].getOrientation() = rot;
	}

	//hduPrintError(stderr, &error, "status!@");
	sofa::simulation::tree::GNode* node = static_cast<sofa::simulation::tree::GNode*>(this->getContext());
	node->execute<sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor>();
	node->execute<sofa::simulation::UpdateMappingVisitor>();
	
	while (node->getParent() != NULL)
		node = static_cast<sofa::simulation::tree::GNode*>(node->getParent());
    
    bool button1NewState = (deviceData.m_buttonState1!=0);
    if (button1NewState != button1State)
    {
        button1State = button1NewState;
        if (button1NewState)
        {
			sofa::core::objectmodel::KeypressedEvent ev(key1.getValue());
            node->propagateEvent(&ev);
        }
        else
        {
            sofa::core::objectmodel::KeyreleasedEvent ev(key1.getValue());
            node->propagateEvent(&ev);
        }
    }
    bool button2NewState = (deviceData.m_buttonState2!=0);
    if (button2NewState != button2State)
    {
        button2State = button2NewState;
        if (button2NewState)
        {
            sofa::core::objectmodel::KeypressedEvent ev(key2.getValue());
            node->propagateEvent(&ev);
        }
        else
        {
            sofa::core::objectmodel::KeyreleasedEvent ev(key2.getValue());
            node->propagateEvent(&ev);
        }
    }
    
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
    if (!bDraw.getValue()) return;
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
	p *= scale.getValue();
	p = mrotation*p;
	p += xform.getValue().getCenter();
	//Vec3d p = Vec3d((z-x)*0.5, y, (-x-z)*0.5)*0.5;
	glEnable(GL_LIGHTING);
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, c.ptr());
	glPushMatrix();
	//std::cout << "SPH "<<index<<" "<<x()<<" "<<y()<<" "<<z()<<" "<<radius<<"\n";
	glTranslated(p[0],p[1],p[2]);
//	glutSolidSphere(0.49*scale.getValue(), 16, 8);
	glPopMatrix();
	glDisable(GL_LIGHTING);
	if (currentForce.norm()>0.0001)
	{
		Vec3d f = currentForce; //Vec3d((currentForce[2]-currentForce[0])*0.5,currentForce[1],(-currentForce[0]-currentForce[2])*0.5);
		Vec3d n = f;
		n.normalize();
		glLineWidth(5);
		Vec3d p1,p2;
		p1 = (*sphereModel->getMechanicalState()->getX())[0]+n*0.5*scale.getValue();
		p2 = (*sphereModel->getMechanicalState()->getX())[0]+n*0.5*scale.getValue()+f*2;
		glBegin(GL_LINES);
//			glVertex3dv(p1.ptr());
//			glVertex3dv(p2.ptr());
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
		Vec3d dp = p0 - pivot.getValue();
		dp -= dir * (dot(dir,dp));
		glLineWidth(5);
		Vec3d p1,p2;
		p1 = pivot.getValue();
		p2 = pivot.getValue()+dp;
		glBegin(GL_LINES);
//			glVertex3dv(p1.ptr());
//			glVertex3dv(p2.ptr());
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
HDCallbackCode HDCALLBACK applyForceCallback(void * /* pUserData */)
{
	static int lastButtons = 0;
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice());
    // Retrieve the current button(s). 
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    // In order to get the specific button 1 state, we use a bitmask to
    // test for the HD_DEVICE_BUTTON_1 bit. 
    gServoDeviceData.m_buttonState1 = (nButtons & HD_DEVICE_BUTTON_1) ? true : false;
//	gServoDeviceData.m_buttonState2 = (nButtons & HD_DEVICE_BUTTON_2) ? true : false;
	gServoDeviceData.m_buttonState2 = (nButtons & ~HD_DEVICE_BUTTON_1) ? true : false;
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
	
	if ((rot[0]*gServoDeviceData.quat[0]+rot[1]*gServoDeviceData.quat[1]+rot[2]*gServoDeviceData.quat[2]+rot[3]*gServoDeviceData.quat[3]) < 0)
	{
		for (int i=0;i<4;i++)
			rot[i] *= -1;
	}	
	
	gServoDeviceData.quat[0] += rot[0];
	gServoDeviceData.quat[1] += rot[1];
	gServoDeviceData.quat[2] += rot[2];
	gServoDeviceData.quat[3] += rot[3];

	//if ( (nButtons & HD_DEVICE_BUTTON_2) && !(lastButtons & HD_DEVICE_BUTTON_2) ) {
	//	bRenderPivot = !bRenderPivot;
	//}

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
				bRenderForce = false;
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
    static HDboolean bRenderForce = false;
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
        bRenderForce = true;
    }
    else if ((nCurrentButtons & HD_DEVICE_BUTTON_1) == 0 &&
             (nLastButtons & HD_DEVICE_BUTTON_1) != 0)

    {
        // Detected button up 
        bRenderForce = false;

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
            bRenderForce = false;
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
