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
#ifndef SOFA_GUI_FLTK_FLTKVIEWER_H
#define SOFA_GUI_FLTK_FLTKVIEWER_H

#include <FL/Fl.H>
#include <stdlib.h>
#include <FL/glut.H>
#include <GL/glu.h>
#ifdef WIN32
#include <GL/glaux.h>
#endif
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <FL/Fl_Gl_Window.H>

// #include <sofa/core/VisualModel/OBJmodel.h>

//for debugging
//#include <sofa/core/VisualModel/SPHviewer.h>
//#include <sofa/core/componentmodel/behavior/MassViewer.h>
////
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/gl/Transformation.h>
#include <sofa/helper/gl/Trackball.h>
#include <sofa/helper/gl/Texture.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/component/collision/RayPickInteractor.h>
#include <sofa/simulation/tree/xml/Element.h>

namespace sofa
{

namespace gui
{

namespace fltk
{

//using namespace sofa::Components;
using namespace sofa::defaulttype;
using namespace sofa::helper::gl;
using namespace sofa::helper::system::thread;
using namespace sofa::simulation::tree;
using namespace sofa::component::collision;

#define TRACKBALL_MODE     1
#define PAN_MODE		   2
#define ZOOM_MODE   	   3
#define MINMOVE 		  10
#define SMOOTHING_ANGLE   90.0
#define WELD_DISTANCE      0.00001
#define SQR(x)  		((x) * (x))
#define SGN(x)  		((x) < 0 ? (-1) : 1)
#define ABS(x)  		((x) < 0 ? (-(x)) : (x))
#define MIN(x,y)		((x) < (y) ? (x) : (y))
#define MAX(x,y)		((x) < (y) ? (y) : (x))

// Interaction
#define XY_TRANSLATION 1
#define Z_TRANSLATION 2


class  FLTKviewer:public Fl_Gl_Window
{
  private:

	int				_W, _H;
	int				_clearBuffer;
	bool			_lightModelTwoSides;
	float			_lightPosition[4];
	int				_navigationMode;
	Trackball		_currentTrackball;
	Trackball		_newTrackball;
//	Quaternion		_currentQuat;
//	Quaternion		_newQuat;
	int				_mouseX, _mouseY;
	int				_savedMouseX, _savedMouseY;
	bool			_spinning;
	bool			_moving;
	float			_zoomSpeed;
	float			_panSpeed;
	Transformation	_sceneTransform;
	Vector3			_previousEyePos;
	GLUquadricObj*	_arrow;
	GLUquadricObj*	_tube;
	GLUquadricObj*	_sphere;
	GLUquadricObj*	_disk;
	char			_filename[2048];
	GLuint			_numOBJmodels;
	GLuint			_materialMode;
	GLboolean		_facetNormal;
	float			_zoom;
	int				_renderingMode;
	//GLuint			_logoTexture;
	Texture			*texLogo;
	bool			_automateDisplayed;
	ctime_t			_beginTime;
	RayPickInteractor* interactor;
	double          lastProjectionMatrix[16];
	double          lastModelviewMatrix[16];
	GLint           lastViewport[4];
	GNode*          groot;
public:

					FLTKviewer(int X, int Y, int w, int h, const char* l = 0);
					~FLTKviewer();

	void			SetScene(GNode* root);
	void			ResetScene(void);
	void			SwitchToPresetView();
	void			SwitchToAutomateView();
	void			Animate(void);
	void			reshape(int width, int height);
	int GetWidth()
	{
		return _W;
	};
	int GetHeight()
	{
		return _H;
	};
	GNode* GetScene()
	{
		return groot;
	}

	void	UpdateOBJ(void);

	//Added by Xunlei wu, 2005
	static bool		_GLContextInitialized;

	//Debugging interface
	/* void 			SetOBJmodel(OBJmodel *model, bool drawable);
	   void 			setSPHviewer(SPHviewer *pSPHviewer, bool drawable); 
	   void 				 setSpringMassViewer (SpringMassViewer *smViewer, bool drawable);
	   void 			setMappingViewer (MappingViewer *mapViewer, bool drawable);
	 */

	void	draw(void);	// required by FLTK

	
	/////////////////
	// Interaction //
	/////////////////

	bool _mouseInteractorTranslationMode;
	bool _mouseInteractorRotationMode;
	bool _mouseInteractorMoving;
	int _mouseInteractorSavedPosX;
	int _mouseInteractorSavedPosY;
	int _translationMode;
	Quaternion _mouseInteractorCurrentQuat;
	Vector3 _mouseInteractorAbsolutePosition;
	Trackball _mouseInteractorTrackball;
	void ApplyMouseInteractorTransformation(int x, int y);

	static Quaternion _mouseInteractorNewQuat;
	static Vector3 _mouseInteractorRelativePosition;
	static Quaternion _newQuat;
	static Quaternion _currentQuat;
	static bool _mouseTrans;
	static bool _mouseRotate;

#ifdef WIN32
	static HANDLE mutex;
#else
	static pthread_mutex_t mutex;
#endif

	/////////////////////////////////////////////////////////////

  private:

	void	InitGFX(void);
	void	PrintString(void* font, char* string);
	void	Display3DText(float x, float y, float z, char* string);
	void	DrawAxis(double xpos, double ypos, double zpos, double arrowSize);
	void	DrawXYPlane(double zo, double xmin, double xmax, double ymin,
						double ymax, double step);
	void	DrawYZPlane(double xo, double ymin, double ymax, double zmin,
						double zmax, double step);
	void	DrawXZPlane(double yo, double xmin, double xmax, double zmin,
						double zmax, double step);
	void	CreateOBJmodelDisplayList(int material_mode);
	//int     loadBMP(char *filename, TextureImage *texture);
	//void	LoadGLTexture(char *Filename);
	void	DrawLogo(void);
	void	DisplayOBJs(void);
	void	DisplayMenu(void);
	void	DrawScene();
	void	DrawAutomate();
	void	ApplySceneTransformation(int x, int y);
	int		handle(int event);	// required by FLTK
};

} // namespace fltk

} // namespace gui

} // namespace sofa

#endif
