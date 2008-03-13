/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the Free   *
* Software Foundation; either version 2 of the License, or (at your option)    *
* any later version.                                                           *
*                                                                              *
* This program is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for     *
* more details.                                                                *
*                                                                              *
* You should have received a copy of the GNU General Public License along with *
* this program; if not, write to the Free Software Foundation, Inc., 51        *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                    *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <math.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <FL/glut.H>
#include <FL/fl_draw.H>
#include <GL/glext.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/simulation/tree/Simulation.h>
#include <GL/glu.h>
#ifdef WIN32
#include <GL/glaux.h>
#endif
#include "FLTKviewer.h"
#include "GUI.h"
#include "Callbacks.h"
#include "Main.h"

#include <sofa/helper/gl/glfont.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/io/ImageBMP.h>

#include <sofa/simulation/automatescheduler/Automate.h>
#include <sofa/simulation/automatescheduler/CPU.h>
#include <sofa/simulation/tree/xml/Element.h>
#include <sofa/simulation/automatescheduler/Edge.h>
#include <sofa/simulation/automatescheduler/Node.h>
#include <sofa/simulation/tree/Simulation.h>
#include <sofa/helper/system/thread/CTime.h>

namespace sofa
{

namespace gui
{

namespace fltk
{

using std::cout;
using std::endl;
using namespace sofa::defaulttype;
using namespace sofa::simulation::automatescheduler;

extern UserInterface*	GUI;
//extern OBJmodel*		cubeModel;


//Added by Xunlei Wu, 2005
//No static variable initialized in constructor!!!
bool FLTKviewer::_GLContextInitialized = false;

// Mouse Interactor
bool FLTKviewer::_mouseTrans = false;
bool FLTKviewer::_mouseRotate = false;
Quaternion FLTKviewer::_mouseInteractorNewQuat;
Quaternion FLTKviewer::_newQuat;
Quaternion FLTKviewer::_currentQuat;
Vector3 FLTKviewer::_mouseInteractorRelativePosition(0,0,0);

#ifdef WIN32
HANDLE FLTKviewer::mutex;
#else
pthread_mutex_t FLTKviewer::mutex;
#endif
//

// ---------------------------------------------------------
// --- Constructor
// ---------------------------------------------------------
FLTKviewer::FLTKviewer(int X, int Y, int w, int h, const char* l): Fl_Gl_Window(X, Y, w, h, l)
{
	// setup OpenGL mode for the window
	Fl_Gl_Window::mode(FL_RGB | FL_DOUBLE | FL_DEPTH | FL_ALPHA);

	_previousEyePos = Vector3(0.0, 0.0, 0.0);
	_zoom = 1.0;
	_zoomSpeed = 250.0;
	_panSpeed = 25.0;
	_navigationMode = TRACKBALL_MODE;
	_spinning = false;
	_moving = false;
	_numOBJmodels = 0;
	_materialMode = 0;
	_facetNormal = GL_FALSE;
	_renderingMode = GL_RENDER;

	_automateDisplayed = false;

	/*_surfaceModel = NULL;
	_springMassView = NULL;
	_mapView = NULL;
	sphViewer = NULL;
	*/
	_arrow = gluNewQuadric();
	gluQuadricDrawStyle(_arrow, GLU_FILL);
	gluQuadricOrientation(_arrow, GLU_OUTSIDE);
	gluQuadricNormals(_arrow, GLU_SMOOTH);

	_tube = gluNewQuadric();
	gluQuadricDrawStyle(_tube, GLU_FILL);
	gluQuadricOrientation(_tube, GLU_OUTSIDE);
	gluQuadricNormals(_tube, GLU_SMOOTH);

	_sphere = gluNewQuadric();
	gluQuadricDrawStyle(_sphere, GLU_FILL);
	gluQuadricOrientation(_sphere, GLU_OUTSIDE);
	gluQuadricNormals(_sphere, GLU_SMOOTH);

	_disk = gluNewQuadric();
	gluQuadricDrawStyle(_disk, GLU_FILL);
	gluQuadricOrientation(_disk, GLU_OUTSIDE);
	gluQuadricNormals(_disk, GLU_SMOOTH);

	// init trackball rotation matrix / quaternion
	_newTrackball.ComputeQuaternion(0.0, 0.0, 0.0, 0.0);
	_newQuat = _newTrackball.GetQuaternion();

	////////////////
	// Interactor //
	////////////////

	_mouseInteractorMoving = false;
	_mouseInteractorTranslationMode = false;
	_mouseInteractorRotationMode = false;
	_mouseInteractorSavedPosX = 0;
	_mouseInteractorSavedPosY = 0;
	_mouseInteractorTrackball.ComputeQuaternion(0.0, 0.0, 0.0, 0.0);
	_mouseInteractorNewQuat = _mouseInteractorTrackball.GetQuaternion();

	interactor = NULL;

#ifdef WIN32
	mutex = CreateMutex( NULL, FALSE, NULL);
#else
	pthread_mutex_init(&mutex, NULL);
#endif

}


// ---------------------------------------------------------
// --- Destructor
// ---------------------------------------------------------
FLTKviewer::~FLTKviewer()
{
}

void FLTKviewer::SetScene(GNode* root)
{
	groot = root;
}

// -----------------------------------------------------------------
// --- OpenGL initialization method - includes light definitions, 
// --- color tracking, etc. This method MUST be called outside of 
// --- the constructor since the OpenGL context is not yet created
// --- at that time. Best place to call is from ::draw()
// -----------------------------------------------------------------
void FLTKviewer::InitGFX(void)
{
	static GLfloat	lightPos[4];
	static GLfloat	specref[4];
	static GLfloat	ambientLight[4];
	static GLfloat	diffuseLight[4];
	static GLfloat	specular[4];
	static GLfloat	lmodel_ambient[]	={0.0f, 0.0f, 0.0f, 0.0f};
	static GLfloat	lmodel_twoside[]	={GL_FALSE};
	static GLfloat	lmodel_local[]		={GL_FALSE};
	static bool		initialized			= false;

	if (!initialized)
	{
                sofa::helper::system::SetDirectory cwd(progname);
		// Define light parameters
		_lightPosition[0] = 0.0f;
		_lightPosition[1] = 100.0f;
		_lightPosition[2] = 0.0f;
		_lightPosition[3] = 1.0f;

		ambientLight[0] = 0.5f;
		ambientLight[1] = 0.5f;
		ambientLight[2] = 0.5f;
		ambientLight[3] = 1.0f;

		diffuseLight[0] = 0.9f;
		diffuseLight[1] = 0.9f;
		diffuseLight[2] = 0.9f;
		diffuseLight[3] = 1.0f;

		specular[0] = 1.0f;
		specular[1] = 1.0f;
		specular[2] = 1.0f;
		specular[3] = 1.0f;

		specref[0] = 1.0f;
		specref[1] = 1.0f;
		specref[2] = 1.0f;
		specref[3] = 1.0f;

		_clearBuffer = GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT;
		_lightModelTwoSides = false;

		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0);
		glEnable(GL_NORMALIZE);

		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

		// Set light model
		glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, lmodel_local);
		glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);
		glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

		// Setup 'light 0'
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
		glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

		// Enable color tracking
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

		// All materials hereafter have full specular reflectivity with a high shine
		glMaterialfv(GL_FRONT, GL_SPECULAR, specref);
		glMateriali(GL_FRONT, GL_SHININESS, 128);

		glShadeModel(GL_SMOOTH);

		// Define background color
		glClearColor(0.0589f, 0.0589f, 0.0589f, 1.0f);

		//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		//Load texture for logo
		// LoadGLTexture("../Data/SOFA_logo.bmp");
                texLogo = new Texture(new sofa::helper::io::ImageBMP("../Data/SOFA_logo.bmp"));
		texLogo->init();

		//Added by Xunlei Wu, 2005
		FLTKviewer::_GLContextInitialized = true;
		
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		// If an OBJ model is specified, create display list
		//---------------------------------------------------
		Simulation::initTextures(groot);
		//---------------------------------------------------
		/* if (_surfaceModel)
		   CreateOBJmodelDisplayList(1);
		*/

		// switch to preset view
		SwitchToPresetView();

		// change status so we only do this stuff once
		initialized = true;

		_beginTime = CTime::getTime();

		printf("\n");
	}
	//sofa::Components::GL::glfntInit();
}

// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::PrintString(void* font, char* string)
{
	int	len, i;

	len = (int) strlen(string);
	for (i = 0; i < len; i++)
	{
		glutBitmapCharacter(font, string[i]);
	}
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::Display3DText(float x, float y, float z, char* string)
{
	char*	c;

	glPushMatrix();
	glTranslatef(x, y, z);
	for (c = string; *c != '\0'; c++)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
	}
	glPopMatrix();
}


// ---------------------------------------------------
// --- 
// --- 
// ---------------------------------------------------
void FLTKviewer::DrawAxis(double xpos, double ypos, double zpos,
						  double arrowSize)
{
	float	fontScale	= (float) (arrowSize / 600.0);

	Enable<GL_DEPTH_TEST> depth;
	Enable<GL_LIGHTING> lighting;
	Enable<GL_COLOR_MATERIAL> colorMat;

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);

	// --- Draw the "X" axis in red
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslated(xpos, ypos, zpos);
	glRotatef(90.0f, 0.0, 1.0, 0.0);
	gluCylinder(_tube, arrowSize / 50.0, arrowSize / 50.0, arrowSize, 10, 10);
	glTranslated(0.0, 0.0, arrowSize);
	gluCylinder(_arrow, arrowSize / 15.0, 0.0, arrowSize / 5.0, 10, 10);
	// ---- Display a "X" near the tip of the arrow
	glTranslated(-0.5 * fontScale * (double)
				 glutStrokeWidth(GLUT_STROKE_ROMAN, 88),
				 arrowSize / 15.0, arrowSize /
							 5.0);
	glLineWidth(3.0);
	glScalef(fontScale, fontScale, fontScale);
	glutStrokeCharacter(GLUT_STROKE_ROMAN, 88);
	glScalef(1.0f / fontScale, 1.0f / fontScale, 1.0f / fontScale);
	glLineWidth(1.0f);
	// --- Undo transforms
	glTranslated(-xpos, -ypos, -zpos);
	glPopMatrix();

	// --- Draw the "Y" axis in green
	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0);
	glTranslated(xpos, ypos, zpos);
	glRotatef(-90.0f, 1.0, 0.0, 0.0);
	gluCylinder(_tube, arrowSize / 50.0, arrowSize / 50.0, arrowSize, 10, 10);
	glTranslated(0.0, 0.0, arrowSize);
	gluCylinder(_arrow, arrowSize / 15.0, 0.0, arrowSize / 5.0, 10, 10);
	// ---- Display a "Y" near the tip of the arrow
	glTranslated(-0.5 * fontScale * (double)
				 glutStrokeWidth(GLUT_STROKE_ROMAN, 89),
				 arrowSize / 15.0, arrowSize /
							 5.0);
	glLineWidth(3.0);
	glScalef(fontScale, fontScale, fontScale);
	glutStrokeCharacter(GLUT_STROKE_ROMAN, 89);
	glScalef(1.0f / fontScale, 1.0f / fontScale, 1.0f / fontScale);
	glLineWidth(1.0);
	// --- Undo transforms
	glTranslated(-xpos, -ypos, -zpos);
	glPopMatrix();

	// --- Draw the "Z" axis in blue
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslated(xpos, ypos, zpos);
	glRotatef(0.0f, 1.0, 0.0, 0.0);
	gluCylinder(_tube, arrowSize / 50.0, arrowSize / 50.0, arrowSize, 10, 10);
	glTranslated(0.0, 0.0, arrowSize);
	gluCylinder(_arrow, arrowSize / 15.0, 0.0, arrowSize / 5.0, 10, 10);
	// ---- Display a "Z" near the tip of the arrow
	glTranslated(-0.5 * fontScale * (double)
				 glutStrokeWidth(GLUT_STROKE_ROMAN, 90),
				 arrowSize / 15.0, arrowSize /
							 5.0);
	glLineWidth(3.0);
	glScalef(fontScale, fontScale, fontScale);
	glutStrokeCharacter(GLUT_STROKE_ROMAN, 90);
	glScalef(1.0f / fontScale, 1.0f / fontScale, 1.0f / fontScale);
	glLineWidth(1.0);
	// --- Undo transforms
	glTranslated(-xpos, -ypos, -zpos);
	glPopMatrix();
}


// ----------------------------------------------------------------------------------
// --- Draw a "plane" in wireframe. The "plane" is parallel to the XY axis
// --- of the main coordinate system 
// ----------------------------------------------------------------------------------
void FLTKviewer::DrawXYPlane(double zo, double xmin, double xmax, double ymin,
							 double ymax, double step)
{
	register double x, y;

	Enable<GL_DEPTH_TEST> depth;

	glBegin(GL_LINES);
	for (x = xmin; x <= xmax; x += step)
	{
		glVertex3d(x, ymin, zo);
		glVertex3d(x, ymax, zo);
	}
	glEnd();

	glBegin(GL_LINES);
	for (y = ymin; y <= ymax; y += step)
	{
		glVertex3d(xmin, y, zo);
		glVertex3d(xmax, y, zo);
	}
	glEnd();
}


// ----------------------------------------------------------------------------------
// --- Draw a "plane" in wireframe. The "plane" is parallel to the XY axis
// --- of the main coordinate system 
// ----------------------------------------------------------------------------------
void FLTKviewer::DrawYZPlane(double xo, double ymin, double ymax, double zmin,
							 double zmax, double step)
{
	register double y, z;
	Enable<GL_DEPTH_TEST> depth;

	glBegin(GL_LINES);
	for (y = ymin; y <= ymax; y += step)
	{
		glVertex3d(xo, y, zmin);
		glVertex3d(xo, y, zmax);
	}
	glEnd();

	glBegin(GL_LINES);
	for (z = zmin; z <= zmax; z += step)
	{
		glVertex3d(xo, ymin, z);
		glVertex3d(xo, ymax, z);
	}
	glEnd();

}


// ----------------------------------------------------------------------------------
// --- Draw a "plane" in wireframe. The "plane" is parallel to the XY axis
// --- of the main coordinate system 
// ----------------------------------------------------------------------------------
void FLTKviewer::DrawXZPlane(double yo, double xmin, double xmax, double zmin,
							 double zmax, double step)
{
	register double x, z;
	Enable<GL_DEPTH_TEST> depth;

	glBegin(GL_LINES);
	for (x = xmin; x <= xmax; x += step)
	{
		glVertex3d(x, yo, zmin);
		glVertex3d(x, yo, zmax);
	}
	glEnd();

	glBegin(GL_LINES);
	for (z = zmin; z <= zmax; z += step)
	{
		glVertex3d(xmin, yo, z);
		glVertex3d(xmax, yo, z);
	}
	glEnd();
}

// -------------------------------------------------------------------
// ---
// -------------------------------------------------------------------
void FLTKviewer::DrawLogo()
{
	int w = 0;
	int h = 0; 
	
	if (texLogo) {
		h = texLogo->getImage()->getHeight();
		w = texLogo->getImage()->getWidth();
	}

	Enable <GL_TEXTURE_2D> tex;
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-0.5, _W, -0.5, _H, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (texLogo)
		texLogo->bind();

	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);
		glTexCoord2d(0.0, 0.0);
		glVertex3d((_W-w)/2, (_H-h)/2, 0.0);

		glTexCoord2d(1.0, 0.0);
		glVertex3d( _W-(_W-w)/2, (_H-h)/2, 0.0);
		
		glTexCoord2d(1.0, 1.0);
		glVertex3d( _W-(_W-w)/2, _H-(_H-h)/2, 0.0);
		
		glTexCoord2d(0.0, 1.0);
		glVertex3d((_W-w)/2, _H-(_H-h)/2, 0.0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);	

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

// -------------------------------------------------------------------
// ---
// -------------------------------------------------------------------
void FLTKviewer::DisplayOBJs(void)
{
	Enable<GL_LIGHTING> light;
	Enable<GL_DEPTH_TEST> depth;

	glShadeModel(GL_SMOOTH);
	//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	Simulation::draw(groot);

	// glDisable(GL_COLOR_MATERIAL);
}


// -------------------------------------------------------
// ---
// -------------------------------------------------------
void FLTKviewer::DisplayMenu(void)
{
	Disable<GL_LIGHTING> light;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-0.5, _W, -0.5, _H, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	glColor3f(0.3f, 0.7f, 0.95f);
	/*if (_surfaceModel != NULL) {
	   glRasterPos2i(10, _H-15);
	   sprintf(buffer,"Model name: %s\n", _surfaceModel->getPathName());
	   PrintString(GLUT_BITMAP_HELVETICA_12, buffer);
	   } */

	glColor3f(0.3f, 0.7f, 0.95f);
	glRasterPos2i(_W / 2 - 5, _H - 15);
	//  	sprintf(buffer,"FPS: %.1f\n", _frameRate.GetFPS());
	//  	PrintString(GLUT_BITMAP_HELVETICA_12, buffer);

	//glTranslated(_W-85.0, 25.0, 0.0);
	//glScaled(4.0, 2.0, 2.0);
	glTranslated(_W-82.0, 40.0, 0.0);
	glScaled(4.0, 4.0, 4.0);
	//DrawLogo();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::DrawScene(void)
{
	DrawLogo();
	_newQuat.buildRotationMatrix(_sceneTransform.rotation);

	glLoadIdentity();
	_sceneTransform.Apply();
	
	glGetDoublev(GL_MODELVIEW_MATRIX,lastModelviewMatrix);
	
	if (_renderingMode == GL_RENDER)
	{
		// Initialize lighting
		glPushMatrix();
		glLoadIdentity();
		glLightfv(GL_LIGHT0, GL_POSITION, _lightPosition);
		glPopMatrix();
		Enable<GL_LIGHT0> light0;

		glColor3f(0.5f, 0.5f, 0.6f);
	//	DrawXZPlane(-4.0, -20.0, 20.0, -20.0, 20.0, 1.0);
	//	DrawAxis(0.0, 0.0, 0.0, 10.0);

		DisplayOBJs();
		
		DisplayMenu();		// always needs to be the last object being drawn
	}
}


void FLTKviewer::DrawAutomate(void)
{
	_newQuat.buildRotationMatrix(_sceneTransform.rotation);

	glLoadIdentity();
	_sceneTransform.Apply();

	for(int i = 0; i < (int) Automate::getInstance()->tabNodes.size(); i++)
	{
		for(int j = 0; j < (int) Automate::getInstance()->tabNodes[i]->tabOutputs.size(); j++)
		{
			Automate::getInstance()->tabNodes[i]->tabOutputs[j]->draw();
		}

		Automate::getInstance()->tabNodes[i]->draw();
	}
}


// ---------------------------------------------------------
// --- Reshape of the window, reset the projection
// ---------------------------------------------------------
void FLTKviewer::reshape(int width, int height)
{
	double	xNear, yNear, zNear, zFar, xOrtho, yOrtho;
	double	xFactor = 1.0, yFactor = 1.0;
	double	offset;
	double	xForeground, yForeground, zForeground, xBackground, yBackground,
			zBackground;

	xNear = 3.5;
	yNear = 3.5;
	zNear = 10.0;
	zFar = 1000.0;
	offset = 0.001;		// for foreground and background planes

	xOrtho = fabs(_sceneTransform.translation[2]) * xNear / zNear;
	yOrtho = fabs(_sceneTransform.translation[2]) * yNear / zNear;

	if ((height != 0) && (width != 0))
	{
		if (height > width)
		{
			xFactor = 1.0;
			yFactor = (double) height / (double) width;
		}
		else
		{
			xFactor = (double) width / (double) height;
			yFactor = 1.0;
		}
	}

	lastViewport[0] = 0;
	lastViewport[1] = 0;
	lastViewport[2] = width;
	lastViewport[3] = height;
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	zForeground = -zNear - offset;
	zBackground = -zFar + offset;

	glFrustum(-xNear * xFactor, xNear * xFactor, -yNear * yFactor,
			  yNear * yFactor, zNear, zFar);
	xForeground = -zForeground * xNear / zNear;
	yForeground = -zForeground * yNear / zNear;
	xBackground = -zBackground * xNear / zNear;
	yBackground = -zBackground * yNear / zNear;

	xForeground *= xFactor;
	yForeground *= yFactor;
	xBackground *= xFactor;
	yBackground *= yFactor;

	glGetDoublev(GL_PROJECTION_MATRIX,lastProjectionMatrix);

	glMatrixMode(GL_MODELVIEW);
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::draw()
{
  //	ctime_t beginDisplay;
	//ctime_t endOfDisplay;

//	beginDisplay = MesureTemps();

	// valid() is turned off when FLTK creates a new context for this window 
	// or when the window resizes, and is turned on after draw() is called.  
	// Use this to avoid unneccessarily initializing the OpenGL context.
	//static double lastOrthoTransZ = 0.0;
	if (!valid())
	{
		InitGFX();		// this has to be called here since we don't know when the context is created
		_W = w();
		_H = h();
		reshape(_W, _H);
	}

	// clear buffers (color and depth)
	glClear(_clearBuffer);
	glClearDepth(1.0);

	if (!_automateDisplayed)
	{
		// draw the scene
		DrawScene();
	}
	else
	{
		DrawAutomate();
	}

	// get elapsed time for measuring FPS
	//  	_frameRate.PingFrameCounter();

	//endOfDisplay = CTime::getTime();

	//ctime_t timeTicks = CTime::getTicksPerSec();
	//displayFPSCB(int((float)timeTicks / (endOfDisplay - _beginTime)));
	//_beginTime = endOfDisplay;
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::Animate(void)
{
	if (_spinning)
	{
		_newQuat = _currentQuat + _newQuat;

	}
	// redraw the entire scene
	damage(1);
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::ApplySceneTransformation(int x, int y)
{
	float	x1, x2, y1, y2;
	float	xshift, yshift, zshift;

	if (_moving)
	{
		if (_navigationMode == TRACKBALL_MODE)
		{
			x1 = (2.0f * _W / 2.0f - _W) / _W;
			y1 = (_H - 2.0f * _H / 2.0f) / _H;
			x2 = (2.0f * (x + (-_mouseX + _W / 2.0f)) - _W) / _W;
			y2 = (_H - 2.0f * (y + (-_mouseY + _H / 2.0f))) / _H;
			_currentTrackball.ComputeQuaternion(x1, y1, x2, y2);
			_currentQuat = _currentTrackball.GetQuaternion();
			_savedMouseX = _mouseX;
			_savedMouseY = _mouseY;
			_mouseX = x;
			_mouseY = y;
			_newQuat = _currentQuat + _newQuat;
			redraw();
		}
		else if (_navigationMode == ZOOM_MODE)
		{
			zshift = (2.0f * y - _W) / _W - (2.0f * _mouseY - _W) / _W;
			_sceneTransform.translation[2] = _previousEyePos[2] -
											 _zoomSpeed * zshift;
			redraw();
		}
		else if (_navigationMode == PAN_MODE)
		{
			xshift = (2.0f * x - _W) / _W - (2.0f * _mouseX - _W) / _W;
			yshift = (2.0f * y - _W) / _W - (2.0f * _mouseY - _W) / _W;
			_sceneTransform.translation[0] = _previousEyePos[0] +
											 _panSpeed * xshift;
			_sceneTransform.translation[1] = _previousEyePos[1] -
											 _panSpeed * yshift;
			redraw();
		}
	}
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void FLTKviewer::ApplyMouseInteractorTransformation(int x, int y)
{
	// Mouse Interaction
	double coeffDeplacement = 0.025;
	Quaternion conjQuat, resQuat, _newQuatBckUp;

	float x1, x2, y1, y2;

	if (_mouseInteractorMoving)
	{

#ifdef WIN32
		WaitForSingleObject(mutex, INFINITE);
#else
		pthread_mutex_lock( &mutex );
#endif

		if (_mouseInteractorRotationMode)
		{
			if ((_mouseInteractorSavedPosX != x) || (_mouseInteractorSavedPosY != y))
			{
				x1 = 0;
				y1 = 0;
				x2 = (2.0f * (x + (-_mouseInteractorSavedPosX + _W / 2.0f)) - _W) / _W;
				y2 = (_H - 2.0f * (y + (-_mouseInteractorSavedPosY + _H / 2.0f))) / _H;

				_mouseInteractorTrackball.ComputeQuaternion(x1, y1, x2, y2);
				_mouseInteractorCurrentQuat = _mouseInteractorTrackball.GetQuaternion();
				_mouseInteractorSavedPosX = x;
				_mouseInteractorSavedPosY = y;
				
				_mouseInteractorNewQuat = _mouseInteractorCurrentQuat + _mouseInteractorNewQuat;
				_mouseRotate = true;
			}
			else
			{
				_mouseRotate = false;
			}

			redraw();
		}
		else if (_mouseInteractorTranslationMode)
		{
			_mouseInteractorAbsolutePosition =  Vector3(0,0,0);
			_mouseInteractorRelativePosition =  Vector3(0,0,0);

			if (_translationMode == XY_TRANSLATION)
			{
				_mouseInteractorAbsolutePosition[0] = coeffDeplacement * (x - _mouseInteractorSavedPosX);
				_mouseInteractorAbsolutePosition[1] = -coeffDeplacement * (y - _mouseInteractorSavedPosY);

				_mouseInteractorSavedPosX = x;
				_mouseInteractorSavedPosY = y;
			}
			else if (_translationMode == Z_TRANSLATION)
			{
				_mouseInteractorAbsolutePosition[2] = coeffDeplacement * (y - _mouseInteractorSavedPosY);

				_mouseInteractorSavedPosX = x;
				_mouseInteractorSavedPosY = y;
			}

			_newQuatBckUp[0] = _newQuat[0];
			_newQuatBckUp[1] = _newQuat[1];
			_newQuatBckUp[2] = _newQuat[2];
			_newQuatBckUp[3] = _newQuat[3];

			_newQuatBckUp.normalize();

			// Conjugate calculation of the scene orientation quaternion
			conjQuat[0] = -_newQuatBckUp[0];
			conjQuat[1] = -_newQuatBckUp[1];
			conjQuat[2] = -_newQuatBckUp[2];
			conjQuat[3] = _newQuatBckUp[3];

			conjQuat.normalize();

			resQuat = _newQuatBckUp.quatVectMult(_mouseInteractorAbsolutePosition) * conjQuat;

			_mouseInteractorRelativePosition[0] = resQuat[0];
			_mouseInteractorRelativePosition[1] = resQuat[1];
			_mouseInteractorRelativePosition[2] = resQuat[2];

			_mouseTrans = true;
			redraw();
		}

#ifdef WIN32	
		ReleaseMutex(mutex);
#else
		pthread_mutex_unlock( &mutex );
#endif

	}
}


// ----------------------------------------
// --- Handle events (mouse, keyboard, ...)
// ----------------------------------------
int FLTKviewer::handle(int event)
{
	int	eventX	= Fl::event_x();
	int	eventY	= Fl::event_y();

	if (Fl_Window::handle(event))
	{
		return 1;
	}

	// --- switch vascular model rendering mode
	if (Fl::event_key('x'))
	{
		// do something here
		return 1;
	}

	// --- switch automate display mode
	if (Fl::event_key('a'))
	{
		if (groot->getMultiThreadSimulation())
		{
			if (!_automateDisplayed)
			{
				_automateDisplayed = true;
				Fl::add_idle(displayAutomateCB);
				SwitchToAutomateView();
                                sofa::helper::gl::glfntInit();
			}
			else
			{
				_automateDisplayed = false;
				Fl::remove_idle(displayAutomateCB);
				SwitchToPresetView();
                                sofa::helper::gl::glfntClose(); 
			}
		}
	}

	// --- 
	if (Fl::get_key(FL_Escape))
	{
		exit(0);
	}

	// --- switch interaction mode
	if (Fl::get_key('c'))
	{
		if (!_mouseInteractorTranslationMode)
		{
			std::cout << "Interaction Mode ON\n";
			_mouseInteractorTranslationMode = true;
			_mouseInteractorRotationMode = false;
		}
		else
		{
			std::cout << "Interaction Mode OFF\n";
			_mouseInteractorTranslationMode = false;
			_mouseInteractorRotationMode = false;
		}	
	}

	if (Fl::get_key(FL_Control_L) || Fl::get_key(FL_Control_R))
	{
		if ((_mouseInteractorTranslationMode) && (!_mouseInteractorRotationMode))
		{
			_mouseInteractorRotationMode = true;
		}
	}
	else
	{
		_mouseInteractorRotationMode = false;
	}

	// ---------------------- Here are the mouse controls for the scene  ----------------------

	if (_mouseInteractorRotationMode)
	{
		switch (event)
		{
			case FL_PUSH:
				// Mouse left button is pushed
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					_mouseInteractorMoving = true;
					_mouseInteractorSavedPosX = eventX;
					_mouseInteractorSavedPosY = eventY;
				}
				break;

			case FL_DRAG:
				// 
				break;

			case FL_RELEASE:
				// Mouse left button is released 
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					if (_mouseInteractorMoving)
					{
						_mouseInteractorMoving = false;
					}
				}

			default:
				break;
		}

		ApplyMouseInteractorTransformation(eventX, eventY);
		Fl_Gl_Window::handle(event);
	}
	else if (_mouseInteractorTranslationMode)
	{
		switch (event)
		{
			case FL_PUSH:
				// Mouse left button is pushed 
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					_translationMode = XY_TRANSLATION;
					_mouseInteractorSavedPosX = eventX;
					_mouseInteractorSavedPosY = eventY;
					_mouseInteractorMoving = true;
				}
				// Mouse right button is pushed 
				else if (Fl::event_button() == FL_RIGHT_MOUSE)
				{
					_translationMode = Z_TRANSLATION;
					_mouseInteractorSavedPosY = eventY;
					_mouseInteractorMoving = true;
				}
				
				break;

			case FL_RELEASE:
				// Mouse left button is released 
				if ((Fl::event_button() == FL_LEFT_MOUSE) && (_translationMode == XY_TRANSLATION))
				{
					if (_mouseInteractorMoving)
					{
						//_mouseInteractorRelativePosition = Vector3::ZERO;
						_mouseInteractorMoving = false;
					}
				}
				// Mouse right button is released 
				else if ((Fl::event_button() == FL_RIGHT_MOUSE) && (_translationMode == Z_TRANSLATION))
				{
					if (_mouseInteractorMoving)
					{
						//_mouseInteractorRelativePosition = Vector3::ZERO;
						_mouseInteractorMoving = false;
					}
				}

			default:
				break;
		}

		ApplyMouseInteractorTransformation(eventX, eventY);
		Fl_Gl_Window::handle(event);
	}
	else if (Fl::get_key(FL_Shift_L) || Fl::get_key(FL_Shift_R))
	{
		_moving = false;
		//_sceneTransform.ApplyInverse();
		if (interactor==NULL)
		{
			interactor = new RayPickInteractor();
			interactor->setName("mouse");
			groot->addObject(interactor);
		}
		interactor->newEvent("show");
		switch (event)
		{
			case FL_PUSH:
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					interactor->newEvent("pick");
				}
				else if (Fl::event_button() == FL_RIGHT_MOUSE)
				{
					interactor->newEvent("pick2");
				}
				break;
			case FL_RELEASE:
				//if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					interactor->newEvent("release");
				}
				break;
		}
		Vector3 p0, px, py, pz;
		gluUnProject(eventX, lastViewport[3]-1-(eventY), 0, lastModelviewMatrix, lastProjectionMatrix, lastViewport, &(p0[0]), &(p0[1]), &(p0[2]));
		gluUnProject(eventX+1, lastViewport[3]-1-(eventY), 0, lastModelviewMatrix, lastProjectionMatrix, lastViewport, &(px[0]), &(px[1]), &(px[2]));
		gluUnProject(eventX, lastViewport[3]-1-(eventY+1), 0, lastModelviewMatrix, lastProjectionMatrix, lastViewport, &(py[0]), &(py[1]), &(py[2]));
		gluUnProject(eventX, lastViewport[3]-1-(eventY), 1, lastModelviewMatrix, lastProjectionMatrix, lastViewport, &(pz[0]), &(pz[1]), &(pz[2]));
		px -= p0;
		py -= p0;
		pz -= p0;
		px.normalize();
		py.normalize();
		pz.normalize();
		Mat4x4d transform;
		transform.identity();
		transform[0][0] = px[0];
		transform[1][0] = px[1];
		transform[2][0] = px[2];
		transform[0][1] = py[0];
		transform[1][1] = py[1];
		transform[2][1] = py[2];
		transform[0][2] = pz[0];
		transform[1][2] = pz[1];
		transform[2][2] = pz[2];
		transform[0][3] = p0[0];
		transform[1][3] = p0[1];
		transform[2][3] = p0[2];
		Mat3x3d mat; mat = transform;
		Quat q; q.fromMatrix(mat);
		//std::cout << p0[0]<<' '<<p0[1]<<' '<<p0[2] << " -> " << pz[0]<<' '<<pz[1]<<' '<<pz[2] << std::endl;
		interactor->newPosition(p0, q, transform);
	}
	else
	{
		if (interactor!=NULL)
			interactor->newEvent("hide");
		switch (event)
		{
			case FL_PUSH:
				// rotate with left button
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					_navigationMode = TRACKBALL_MODE;
					_newTrackball.ComputeQuaternion(0.0, 0.0, 0.0, 0.0);
					_currentQuat = _newTrackball.GetQuaternion();
					_moving = true;
					_spinning = false;
					_mouseX = eventX;
					_mouseY = eventY;
				}

				// translate with middle button (if it exists)
				else if (Fl::event_button() == FL_MIDDLE_MOUSE)
				{
					_navigationMode = PAN_MODE;
					_moving = true;
					_mouseX = eventX;
					_mouseY = eventY;
					_previousEyePos[0] = _sceneTransform.translation[0];
					_previousEyePos[1] = _sceneTransform.translation[1];
				}

				// zoom with right button
				else if (Fl::event_button() == FL_RIGHT_MOUSE)
				{
					_navigationMode = ZOOM_MODE;
					_moving = true;
					_mouseX = eventX;
					_mouseY = eventY;
					_previousEyePos[2] = _sceneTransform.translation[2];
				}
				break;

			case FL_DRAG:
				// 
				break;

			case FL_RELEASE:
				// Mouse left button is released 
				if (Fl::event_button() == FL_LEFT_MOUSE)
				{
					if (_moving && _navigationMode == TRACKBALL_MODE)
					{
						_moving = false;
						if ((ABS(eventX - _savedMouseX) >= MINMOVE) ||
							(ABS(eventY - _savedMouseY) >= MINMOVE))
						{
							_spinning = true;
						}
					}
				}

				// Mouse middle button is released
				else if (Fl::event_button() == FL_MIDDLE_MOUSE)
				{
					if (_moving && _navigationMode == PAN_MODE)
					{
						_moving = false;
					}
				}

				// Mouse right button is released
				else if (Fl::event_button() == FL_RIGHT_MOUSE)
				{
					if (_moving && _navigationMode == ZOOM_MODE)
					{
						_moving = false;
					}
				}

				break;

			case FL_MOUSEWHEEL:
				// it is also possible to zoom with mouse wheel (if it exists)
				if (Fl::event_button() == FL_MOUSEWHEEL)
				{
					_navigationMode = ZOOM_MODE;
					_moving = true;
					_mouseX = 0;
					_mouseY = 0;
					eventX = 0;
					eventY = 10 * Fl::event_dy();
					_previousEyePos[2] = _sceneTransform.translation[2];
				}
				break;

			default:
				break;
		}

		ApplySceneTransformation(eventX, eventY);
		Fl_Gl_Window::handle(event);
	}

	return 1;
}


// -------------------------------------------------------------------
// ---
// -------------------------------------------------------------------
void FLTKviewer::ResetScene(void)
{
	/*
	_sceneTransform.translation[0] = 0.0;
	_sceneTransform.translation[1] = 0.0;
	_sceneTransform.translation[2] = -50.0;
	_currentTrackball.ComputeQuaternion(0.0, 0.0, 0.0, 0.0);
	_currentQuat = _newTrackball.GetQuaternion();
	_newTrackball.ComputeQuaternion(0.0, 0.0, 0.0, 0.0);
	_newQuat = _newTrackball.GetQuaternion();
	*/
	Simulation::reset(groot);
}


// -------------------------------------------------------------------
// ---
// -------------------------------------------------------------------
void FLTKviewer::SwitchToPresetView()
{
	_sceneTransform.translation[0] = 0.0;
	_sceneTransform.translation[1] = 0.0;
	_sceneTransform.translation[2] = -50.0;
	_newQuat[0] = 0.17;
	_newQuat[1] = -0.83;
	_newQuat[2] = -0.26;
	_newQuat[3] = -0.44;
}


// -------------------------------------------------------------------
// ---
// -------------------------------------------------------------------
void FLTKviewer::SwitchToAutomateView()
{
	_sceneTransform.translation[0] = -10.0;
	_sceneTransform.translation[1] = 0.0;
	_sceneTransform.translation[2] = -50.0;
	_newQuat[0] = 0.0;
	_newQuat[1] = 0.0;
	_newQuat[2] = 0.0;
	_newQuat[3] = 0.0;
}

} // namespace fltk

} // namespace gui

} // namespace sofa
