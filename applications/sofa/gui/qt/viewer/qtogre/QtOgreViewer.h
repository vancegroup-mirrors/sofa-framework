/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef __sofaOgreWidget_H__
#define __sofaOgreWidget_H__

#include <stdlib.h>

#include <viewer/qtogre/DotSceneLoader.h>

#include <viewer/SofaViewer.h>
#include <sofa/helper/gl/Capture.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/tree/Simulation.h>


#ifdef SOFA_QT4
#include <QPaintEvent>
#include <QWidget>
#else
#include <qlayout.h>
#endif

#include <Ogre.h>



using namespace sofa::helper::system::thread;
using namespace sofa::simulation::tree;

#if defined(SOFA_GPU_CUDA)
#include <sofa/gpu/cuda/mycuda.h>
using namespace sofa::gpu::cuda;
#endif


#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
std::string macBundlePath();
#endif


//*********************************************************************************//
// Widget with Ogre embedded
//*********************************************************************************//

namespace sofa 
{

  namespace gui
  {  

    namespace qt
    {

      namespace viewer
      {

	namespace qtogre
	{

	  class QtOgreViewer : public QWidget, public sofa::gui::qt::viewer::SofaViewer
	    {
	      Q_OBJECT
		public:

	      std::string sceneName;


	      /// Activate this class of viewer.
	      /// This method is called before the viewer is actually created
	      /// and can be used to register classes associated with in the the ObjectFactory.
	      static int EnableViewer();

	      /// Disable this class of viewer.
	      /// This method is called after the viewer is destroyed
	      /// and can be used to unregister classes associated with in the the ObjectFactory.
	      static int DisableViewer();

	      QtOgreViewer( QWidget *parent=0, const char *name=0 );
	      ~QtOgreViewer()
		{

		  if(mRoot != NULL){
		    delete mRoot;
		    mRoot = NULL;
		  }
		};

	      QWidget* getQWidget() { return this; }

	      bool ready(){return _waitForRender;};
	      void wait(){_waitForRender = true;};/*
						    virtual void update(void);*/
	      void showEntireScene(void);


	      void setScene(sofa::simulation::tree::GNode* scene, const char* filename, bool keepParams=false);

	      void setup(void);
	      void setupView(void);          //Creation of the first window of visualization
	      QString helpString();

	      void moveRayPickInteractor(int eventX, int eventY);
	      
	    private:
				
	      Ogre::String mResourcePath;
	      //------------------------------------------
	      //Setup
	      Ogre::RenderWindow* mRenderWindow;
	      Ogre::SceneManager* mSceneMgr;
	      //  SofaListener* mFrameListener;
	      Ogre::Camera* mCamera;
	      Ogre::Viewport* mVp;
	      Ogre::Root* mRoot;
	      Ogre::ShadowTechnique shadow;
	      ///////
	      bool _mouseInteractorMoving;
	      int _mouseInteractorSavedPosX;
	      int _mouseInteractorSavedPosY;
	      unsigned int number_visualModels;
	      //sofa::helper::gl::Capture capture;

	      void setupResources(void);     //Ogre Initialization 

	      void setupConfiguration(void); //Graphic configuration


	      void loadResources(void)       //Launch the Resource Manager
	      {
		// Initialise, parse scripts etc
		Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	      }

	      void createScene(void);        

	      //******************************Configuration Panel for Ogre***********************************
	      virtual bool configure(void)
	      {
		// Show the configuration dialog and initialise the system
		// You can skip this and use root.restoreConfig() to load configuration
		// settings if you were sure there are valid ones saved in ogre.cfg
		std::cerr<<"Show Dialog " << mRoot<<"\n";
		if(mRoot->showConfigDialog())
		  {
		    // Custom option - to use PlaneOptimalShadowCameraSetup we must have
		    // double-precision. Thus, set the D3D floating point mode if present, 
		    // no matter what was chosen
		    Ogre::ConfigOptionMap& optMap = mRoot->getRenderSystem()->getConfigOptions();
		    Ogre::ConfigOptionMap::iterator i = optMap.find("Floating-point mode");
		    if (i != optMap.end())
		      {
			if (i->second.currentValue != "Consistent")
			  {
			    i->second.currentValue = "Consistent";
			    Ogre::LogManager::getSingleton().logMessage("ExampleApplication: overriding "
									"D3D floating point mode to 'Consistent' to ensure precision "
									"for numerical computations");
			  }
		      }
		    // If returned true, user clicked OK so initialise

		    mRoot->initialise(false, "SOFA - OGRE");
		    return true;
		  }
		else
		  {
		    return false;
		  }
	      }

	      void update();

	    protected:

	      ctime_t _beginTime;

	      bool m_mouseLeftPressed;
	      bool m_mouseRightPressed;
	      bool m_mouseMiddlePressed;
	      QPoint m_mousePressPos;
	      QPoint m_mousePos;
	      bool m_background;
	      bool pickDone;

	      Ogre::Vector3 m_mTranslateVector;
	      Ogre::Radian m_mRotX, m_mRotY;
	      Ogre::Real m_mMoveSpeed;
	      Ogre::Degree m_mRotateSpeed;
	      Ogre::SceneNode* zeroNode;
	      Ogre::SceneNode*  camNode;
	      bool _waitForRender;

	      //Initial Bounding Box
	      Ogre::AxisAlignedBox world_BB;

	      void resize()
	      {
		if (mRenderWindow != NULL)
		  {
		    emit(resizeW(width())); 
		    emit(resizeH(height())); 
		    mRenderWindow->windowMovedOrResized();
		    mRenderWindow->resize(width(), height());     
		    mVp->setDimensions(0,0, 1.0, 1.0);
		    mCamera->setAspectRatio(Ogre::Real(mVp->getActualWidth()) / Ogre::Real(mVp->getActualHeight()));
		    update();
		  }
	      }

	      bool updateInteractor( QMouseEvent * e );	
				
	      virtual void paintEvent(QPaintEvent*);				
	      virtual void resizeEvent(QResizeEvent*);			
	      virtual void timerEvent(QTimerEvent * event){Q_UNUSED(event);update();}

	      virtual void keyPressEvent ( QKeyEvent * e );
	      virtual void mousePressEvent(QMouseEvent* evt);
	      virtual void mouseReleaseEvent(QMouseEvent* evt);
	      virtual void mouseMoveEvent(QMouseEvent* evt);
	      virtual void wheelEvent(QWheelEvent* evt); 
			
	      void moveCamera(void)
	      {

		mCamera->yaw(m_mRotX);
		mCamera->pitch(m_mRotY);

		//Reset to zero
		m_mRotX = m_mRotY = Ogre::Degree(0);
		m_mTranslateVector = Ogre::Vector3::ZERO;
	      }


	      public slots:	      
	      virtual void resetView();
	      virtual void saveView();  
	      virtual void setSizeW(int); 
	      virtual void setSizeH(int);

	    signals:
	      void redrawn(); 
	      void resizeW( int ); 
	      void resizeH( int ); 


	    };
	} //qtogre
      } //viewer
    } //qt
  } //gui
} //sofa


#endif

