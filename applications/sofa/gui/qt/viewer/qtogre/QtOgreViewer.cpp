#include <OgreConfigFile.h>
#include "viewer/qtogre/QtOgreViewer.h"
#include "viewer/qtogre/DotSceneLoader.h"
#include "viewer/qtogre/OgreVisualModel.h"

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/glu.h>

#include <sofa/simulation/automatescheduler/ThreadSimulation.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/ObjectFactory.h>

#include <qapplication.h>

#ifdef __linux__
#include <X11/Xlib.h>
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <CoreFoundation/CoreFoundation.h>
// This function will locate the path to our application on OS X,
// unlike windows you can not rely on the curent working directory
// for locating your configuration files and resources.
std::string macBundlePath()
{
	char path[1024];
	CFBundleRef mainBundle = CFBundleGetMainBundle();
	assert(mainBundle);
	
	CFURLRef mainBundleURL = CFBundleCopyBundleURL(mainBundle);
	assert(mainBundleURL);
	
	CFStringRef cfStringRef = CFURLCopyFileSystemPath( mainBundleURL, kCFURLPOSIXPathStyle);
	assert(cfStringRef);
	
	CFStringGetCString(cfStringRef, path, 1024, kCFStringEncodingASCII);
	
	CFRelease(mainBundleURL);
	CFRelease(cfStringRef);
	
	return std::string(path);
}
#endif 

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

	  SOFA_LINK_CLASS(OgreVisualModel) 

	  using sofa::simulation::tree::Simulation;

	  static bool enabled = false;
	  sofa::core::ObjectFactory::ClassEntry* classOglModel;
	  sofa::core::ObjectFactory::ClassEntry* classVisualModel;

	  /// Activate this class of viewer.
	  /// This method is called before the viewer is actually created
	  /// and can be used to register classes associated with in the the ObjectFactory.
	  int QtOgreViewer::EnableViewer()
	  {
	    if (!enabled)
	      {
		enabled = true;
		// Replace OpenGL visual models with OgreVisualModel
		sofa::core::ObjectFactory::AddAlias("OglModel", "OgreVisualModel", true, &classOglModel); 
		sofa::core::ObjectFactory::AddAlias("VisualModel", "OgreVisualModel", true, &classVisualModel); 
	      }
	    return 0;
	  }

	  /// Disable this class of viewer.
	  /// This method is called after the viewer is destroyed
	  /// and can be used to unregister classes associated with in the the ObjectFactory.
	  int QtOgreViewer::DisableViewer()
	  {
	    if (enabled)
	      {
		enabled = false;
		// Replace OpenGL visual models with OgreVisualModel
		sofa::core::ObjectFactory::ResetAlias("OglModel", classOglModel); 
		sofa::core::ObjectFactory::ResetAlias("VisualModel", classVisualModel); 
	      }
	    return 0;
	  }

	  //Application principale
	  QtOgreViewer::QtOgreViewer( QWidget *parent, const char *name )
	    : QWidget( parent, name )
	  { 
			  
#ifdef QT_MODULE_QT3SUPPORT
	    setUpdatesEnabled(false);
	    setWindowFlags(Qt::WindowStaysOnTopHint);
#else
	    setWFlags(Qt::WNoAutoErase);
	    setWFlags(Qt::WStyle_StaysOnTop);
#endif      
	
	

 
	    // Make sure this class is enabled
	    EnableViewer();

	    //*************************************************************************
	    // Ogre Init
	    //*************************************************************************//
	    mRenderWindow = NULL;
	    mSceneMgr = NULL;
	    mVp = NULL;
	    shadow = Ogre::SHADOWTYPE_STENCIL_ADDITIVE;

	    //*************************************************************************
	    // Interface Init
	    //*************************************************************************//
	    _waitForRender = false;

	    capture.setCounter();
	    m_isControlPressed = false;
	    m_mouseLeftPressed = false;
	    m_mouseRightPressed = false;
	    m_mouseMiddlePressed = false;
	    m_mousePos = QPoint(0, 0);
	    m_mousePressPos = QPoint(0, 0);
	    m_mTranslateVector = Ogre::Vector3::ZERO;
	    m_mRotX = m_mRotY = 0.0f;
	    m_mMoveSpeed = 10;
	    m_mRotateSpeed = 36;
	    m_background = true;
	    interactor = NULL;
	    _mouseInteractorMoving = false;
	    _mouseInteractorSavedPosX = 0;
	    _mouseInteractorSavedPosY = 0;
	    number_visualModels=0;				
	    _video = false;

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
	    mResourcePath = macBundlePath() + "/Contents/Resources/";
#else
	    mResourcePath = "";
#endif

	    //Default files
	    sofa::helper::system::FileRepository repository;

	    sceneName = "default.scene";

#if defined(SOFA_GPU_CUDA)
	    mycudaInit(0);
#endif
	  }


	  void QtOgreViewer::setup()
	  {
	    setupResources();

	    setupConfiguration();
	    //if(!configure()) {std::cout << "No configuration \a\n";exit(0);}

	    setupView();

	    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	    loadResources();

	    //createScene();
	  }


	  //*****************************************************************************************
	  //Setup Ogre
	  void QtOgreViewer::setupResources()
	  {

	    Ogre::String pluginsPath;
	    Ogre::String configPath;
	    Ogre::String ogrePath;
	    Ogre::String ogreLog;
	    // only use plugins.cfg if not static
#ifndef OGRE_STATIC_LIB


	    ogrePath = sofa::helper::system::DataRepository.getFile("config/ogre.cfg");
	    ogreLog = sofa::helper::system::DataRepository.getFile("config/Ogre.log");

#ifdef WIN32
	    pluginsPath = sofa::helper::system::DataRepository.getFile("config/plugins.cfg");
#else
	    pluginsPath = sofa::helper::system::DataRepository.getFile("config/plugins_unix.cfg");
#endif

#endif
	    mRoot = new Ogre::Root(pluginsPath, ogrePath, ogreLog);
	    std::string default_scene = sofa::helper::system::DataRepository.getFile("default.scene");
	
	    std::ostringstream ofilename;
	    if (default_scene == "default.scene")
	      {
		std::string::size_type position_scene = ogrePath.rfind("share/config/");
		const char* begin = ogrePath.c_str();					
		const char* end = begin + position_scene;
		ofilename << std::string(begin, end);

	      }
	    else
	      {
		std::string::size_type position_scene = default_scene.rfind("scenes/");
		const char* begin = default_scene.c_str();					
		const char* end = begin + position_scene;
		ofilename << std::string(begin, end);
	      }

	    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ofilename.str() +"scenes","FileSystem","General");
	    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ofilename.str() +"share/textures","FileSystem","General");
	    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ofilename.str() +"share/materials","FileSystem","General");
	    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ofilename.str() +"share/shaders","FileSystem","General");
	  }



	  void QtOgreViewer::setupConfiguration(void)
	  {
	    //Rendering Device - will pick first available render device
	    Ogre::RenderSystemList::iterator pRend = mRoot->getAvailableRenderers()->begin();		
	    Ogre::RenderSystem* mRenderSystem = *pRend;

	    //RenderSystem
	    mRoot->setRenderSystem(mRenderSystem);
	    //Anti aliasing
	    mRenderSystem->setConfigOption("Anti aliasing", "None");
	    //Floating-point mode
	    mRenderSystem->setConfigOption("Floating-point mode", "Fastest");
	    //Full Screen
	    mRenderSystem->setConfigOption("Full Screen", "No");
	    //Vsync
	    mRenderSystem->setConfigOption("VSync", "No");		

	    mRenderSystem->validateConfigOptions(); 

	    mRoot->initialise(false, "SOFA - OGRE");
	  }

	  //*****************************************************************************************
	  //called to redraw the window
	  void QtOgreViewer::update()
	  {
	    if(mRenderWindow == NULL)	  
	      setupView();

	    if(mRenderWindow != NULL){
	      mRoot->_fireFrameStarted();

	      if (_video)
		{
#ifdef CAPTURE_PERIOD
		  static int counter = 0;
		  if ((counter++ % CAPTURE_PERIOD)==0)
#endif
		    screenshot(capture.findFilename());
		}

	      moveCamera();

	      mRenderWindow->update();
	      OgreVisualModel::lightSwitched = false;

	      mRoot->_fireFrameEnded();
	      if (_waitForRender) _waitForRender = false;
#ifdef QT_MODULE_QT3SUPPORT
	      setUpdatesEnabled(false);
#endif
	    }
	  }
	  //******************************Qt paint***********************************

	  void QtOgreViewer::paintEvent(QPaintEvent *)
	  {				
	    update();
	    emit( redrawn() );
	  }

	  //*****************************************************************************************
	  //Initialize the rendering window: create a sofa simulation, Ogre window...
	  void QtOgreViewer::setupView()
	  {

	    Ogre::NameValuePairList params;

#if defined(__linux__)

	    // 	params["parentWindowHandle"] =
	    // 	  Ogre::StringConverter::toString ((unsigned long)XOpenDisplay(NULL)) +
	    // 	  ":" + Ogre::StringConverter::toString ((unsigned long)parentWidget()->winId());   



	    Display* display = qt_xdisplay(); //XOpenDisplay(NULL);
// 	    int screen = qt_xscreen(); //DefaultScreen(display);

	    params["parentWindowHandle"] = 
	      Ogre::StringConverter::toString ((unsigned long)display) +
	      // 	      ":" + Ogre::StringConverter::toString ((unsigned long)screen) +
	      ":" + Ogre::StringConverter::toString ((unsigned long)parentWidget()->winId());

	    // 	    params["parentWindowHandle"] = 
	    // 	      Ogre::StringConverter::toString ((unsigned long)display) +
	    // 	      ":" + Ogre::StringConverter::toString ((unsigned long)screen) +
	    // 	      ":" + Ogre::StringConverter::toString ((unsigned long)parentWidget()->winId());



#elif defined (WIN32) 
	    params["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)(HWND)winId());
#elif defined (__APPLE__)

	    // TODO
#endif

	    // 	try{
	    //(name, width, height, fullscreen, parametre)
	    mRenderWindow = mRoot->createRenderWindow("View", width(), height(), false, &params);

	    _beginTime = CTime::getTime();

#if defined(__linux__)
	    WId window_id;
	    mRenderWindow->getCustomAttribute("WINDOW", &window_id);

	    // Take over the ogre created window.
	    QWidget::create(window_id);

	    mRenderWindow->reposition(x(),y());
#elif defined (__APPLE__)
				
	    // TODO
#endif

#ifdef QT_MODULE_QT3SUPPORT
	    startTimer(500);
#endif
	  }




	  //*****************************************************************************************
	  //Initialize Sofa with the scene, and load the components to Ogre

	  void QtOgreViewer::createScene()
	  {  	

	    using namespace Ogre;


	    if (groot==NULL){
	      groot = new GNode;
	    }

	    DotSceneLoader loader;
	    loader.parseDotScene(sceneName, "General", NULL); //mSceneMgr);
	    mSceneMgr = loader.getSceneManager();	


	    mSceneMgr->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);
	    if (groot)
	      {
		// This could be done as an action, but it is shorter to do it this way
		std::vector<OgreVisualModel*> visualModels;
		groot->getTreeObjects<OgreVisualModel>(&visualModels);

		for (unsigned int i=0; i<visualModels.size(); i++)
		  {	      
		    visualModels[i]->attach(mSceneMgr);
		  }
		number_visualModels = visualModels.size();

	      }

	    if (mSceneMgr->hasCamera("sofa_camera"))	
	      mCamera = mSceneMgr->getCamera("sofa_camera");	
	    else
	      {		  
		// Create the camera
		mCamera = mSceneMgr->createCamera("sofa_camera");
		// Position it at 50 in Z direction	
		mCamera->setPosition(Ogre::Vector3(50,100,50));
		// Look back along -Z		
		mCamera->lookAt(Ogre::Vector3(0,-10,0));
		mCamera->setNearClipDistance(loader.environment.nearClipDistance);
		mCamera->setFarClipDistance(loader.environment.farClipDistance);
	      }
	    //Always yaw around the camera Y axis.
	    mCamera->setFixedYawAxis(false);

	    // Create one viewport, entire window
	    if (mVp)
	      {
		mRenderWindow->removeViewport(0);
	      }
	    mVp = mRenderWindow->addViewport(mCamera);
	    mVp->setBackgroundColour(loader.environment.backgroundColour);


	    showEntireScene();

	
	    //************************************************************************************************
	    //insert background sofa
	    if ( !mSceneMgr->hasSceneNode("Background"))
	      {
		//Warning: if scene reloaded, the resource still exists but may have lost its information.
		if (Ogre::MaterialManager::getSingleton().resourceExists("Background"))
		  Ogre::MaterialManager::getSingleton().remove("Background");

		Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Background", "General");

		material->getTechnique(0)->getPass(0)->createTextureUnitState("SOFA_logo.bmp");
		material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setSceneBlending (Ogre::SBT_ADD);

		// Create background rectangle covering the whole screen
		Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
		rect->setCorners(-1.0, 0.5, 1.0, -0.5);
		rect->setMaterial("Background");
	
		// Render the background before everything else
		rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
	
		Ogre::AxisAlignedBox aabInf;
		aabInf.setInfinite();
		rect->setBoundingBox(aabInf);

		// Attach background to the scene
		Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode("Background");
		node->attachObject(rect);
	      }

	    //************************************************************************************************
	    // Alter the camera aspect ratio to match the viewport
	    mCamera->setAspectRatio(Real(mVp->getActualWidth()) / Real(mVp->getActualHeight()));

	  }

	  void QtOgreViewer::showEntireScene()
	  {
	    mSceneMgr->_updateSceneGraph (mCamera);

	    //Verify if there is no new visual model.
	    if (groot)
	      {
		std::vector<OgreVisualModel*> visualModels;
		groot->getTreeObjects<OgreVisualModel>(&visualModels);
		if (visualModels.size() != number_visualModels)
		  {
		    if (visualModels.size() > number_visualModels)
		      {
			//Add only the new visual models
			for ( int i=number_visualModels; i<(int)visualModels.size(); i++)		     
			  visualModels[i]->attach(mSceneMgr);
		      
		      }
		    else
		      {
			//Remove elements  
			mSceneMgr->destroyAllEntities();
			//Add the visual models
			for (unsigned int i=0; i<visualModels.size(); i++)		      
			  visualModels[i]->attach(mSceneMgr);		      

		      }

		    number_visualModels = visualModels.size();
		  }
	      }
	    
	    //************************************************************************************************
	    //Calculate the World Bounding Box
	    
	    sofa::defaulttype::Vector3 sceneMinBBox;
	    sofa::defaulttype::Vector3 sceneMaxBBox;
	    getSimulation()->computeBBox(groot, sceneMinBBox.ptr(), sceneMaxBBox.ptr());

	    Ogre::Vector3 size_world(sceneMaxBBox[0] - sceneMinBBox[0],sceneMaxBBox[1] - sceneMinBBox[1],sceneMaxBBox[2] - sceneMinBBox[2]);
	    float max = ((size_world.x > size_world.y)?size_world.x:size_world.y);
	    max = ((max > size_world.z)?max:size_world.z);
	    float min = ((size_world.x < size_world.y)?size_world.x:size_world.y);
	    min = ((min < size_world.z)?min:size_world.z);


	    //frustrum
	    if (min <= 1) min = 1.0;
// 	    if (max <= min) max = 1000.0*min;

	    mCamera->setFarClipDistance(Ogre::Real( 1000.0+10.0*max));
	    //mCamera->setNearClipDistance(Ogre::Real( 0.1*min)); //cause a crash of wrong BBox


	    //position
	    Ogre::Vector3 camera_position((sceneMaxBBox[0]+sceneMinBBox[0])/2.0f,(sceneMaxBBox[1]+sceneMinBBox[1])/2.0f,(sceneMaxBBox[2]+sceneMinBBox[2])/2.0f);

	    mCamera->setPosition(Ogre::Vector3(camera_position.x, camera_position.y, camera_position.z));
	    mCamera->moveRelative(Ogre::Vector3(0.0,0.0,2*std::max(size_world[0], std::max(size_world[1], size_world[2]))));

	    mCamera->lookAt(camera_position);
	    return;
	  }



	  //*********************************Qt Control*******************************
	  void QtOgreViewer::resizeEvent(QResizeEvent *evt)
	  {
	    Q_UNUSED(evt);
	    resize();
	  }


	  //*****************************************************************************************
	  //Keyboard Events

	  void QtOgreViewer::keyPressEvent ( QKeyEvent * e )
	  {
	    if( isControlPressed() ) // pass event to the scene data structure
	      {
		//cerr<<"QtViewer::keyPressEvent, key = "<<e->key()<<" with Control pressed "<<endl;
		sofa::core::objectmodel::KeypressedEvent keyEvent(e->key());
		groot->propagateEvent(&keyEvent);
	      }
	    else  // control the GUI
	      {
		switch(e->key())
		  {
		  case Qt::Key_B:
		    {
		      m_background = !m_background;
		      Ogre::SceneNode* node = mSceneMgr->getSceneNode("Background");
		      node->setVisible(m_background);
		
		      break;
		    }
		  case Qt::Key_C:
		    {
		      showEntireScene();
		      break;
		    }
		  case Qt::Key_L:
		    // --- Modify the shadow type
		    {
		      switch(shadow)
			{		    
			case Ogre::SHADOWTYPE_NONE:
			  shadow = Ogre::SHADOWTYPE_STENCIL_ADDITIVE;
			  OgreVisualModel::lightSwitched = true;
			  break;
			case Ogre::SHADOWTYPE_STENCIL_ADDITIVE:
			default:
			  shadow = Ogre::SHADOWTYPE_NONE;
			  break;
			}
		      mCamera->getSceneManager()->setShadowTechnique(shadow);
		      break;
		    }
		  case Qt::Key_R:
		    // --- Modify the way the polygons are rendered
		    {
		      switch(mCamera->getPolygonMode())
			{
			case Ogre::PM_POINTS:
			  mCamera->setPolygonMode(Ogre::PM_WIREFRAME);
			  break;

			case Ogre::PM_WIREFRAME:
			  mCamera->setPolygonMode(Ogre::PM_SOLID);
			  break;

			case Ogre::PM_SOLID:
			  mCamera->setPolygonMode(Ogre::PM_POINTS);
			  break;
			}
		      break;
		    }
		  default:
		    {
		      SofaViewer::keyPressEvent(e);
		    }
		  }
	      }
	    update();
	  }


	  //*****************************************************************************************
	  //Mouse Events

	  void QtOgreViewer::mousePressEvent(QMouseEvent* evt)
	  {
	    if (!updateInteractor(evt))
	      {
		m_mousePos = evt->globalPos(); 

		if(evt->button() == Qt::LeftButton)
		  m_mouseLeftPressed = true;
		if(evt->button() == Qt::RightButton)
		  m_mouseRightPressed = true;       
		if(evt->button() == Qt::MidButton)
		  m_mouseMiddlePressed = true;
	      }

	  }


	  void QtOgreViewer::mouseReleaseEvent(QMouseEvent* evt)
	  {
	    if (!updateInteractor(evt))
	      {
		m_mousePos = evt->globalPos(); 
		Q_UNUSED(evt);
		m_mouseLeftPressed = false;
		m_mouseRightPressed = false;
		m_mouseMiddlePressed = false;
		m_mRotX = m_mRotY = Ogre::Degree(0);
		m_mTranslateVector = Ogre::Vector3::ZERO;
	      }

	    if (interactor!=NULL)
	      interactor->newEvent("hide");
	    
	  }

	  void QtOgreViewer::mouseMoveEvent(QMouseEvent* evt)
	  {

	    if (!updateInteractor(evt))
	      {
		if( m_mouseLeftPressed )
		  {
		    m_mRotX = Ogre::Degree( -(evt->globalX() - m_mousePos.x()) * 0.045);
		    m_mRotY = Ogre::Degree( -(evt->globalY() - m_mousePos.y()) * 0.045);
		  }
		if( m_mouseMiddlePressed )
		  {
		    m_mTranslateVector.z += (evt->globalY() - m_mousePos.y()) * 0.045;
		  }
		if( m_mouseRightPressed )
		  {
		    m_mTranslateVector.x +=  (evt->globalX() - m_mousePos.x()) * 0.045/3.0;
		    m_mTranslateVector.y += -(evt->globalY() - m_mousePos.y()) * 0.045/3.0;
		  }

		m_mousePos = evt->globalPos(); 
		if (m_mouseLeftPressed || m_mouseMiddlePressed ||  m_mouseRightPressed ) update();
		  
	      }
	  }


	  void QtOgreViewer::wheelEvent(QWheelEvent* evt)
	  {
	    m_mTranslateVector.z +=  evt->delta()* 0.009;
	    update();
	  }



	  void QtOgreViewer::setScene(sofa::simulation::tree::GNode* scene, const char* filename, bool keepParams)
	  {
	    //Reset at initialization
	    interactor = NULL;

	    capture.setCounter();
	    std::ostringstream ofilename;
	    std::string screenshot_prefix;
	    sceneFileName = (filename==NULL)?"":filename;

	    if (!sceneFileName.empty())
	      {
		const char* begin = sceneFileName.c_str();
		const char* end = strrchr(begin,'.');
		if (!end) end = begin + sceneFileName.length();
		ofilename << std::string(begin, end);
		ofilename << "_";

		std::string sceneNameTemp(std::string(begin, end) + std::string(".scene"));
		std::ifstream in(sceneNameTemp.c_str());
		if (!in.fail())
		  {
		    sceneName = sceneNameTemp;
		    in.close();
		  }

		screenshot_prefix = ofilename.str();
		std::string::size_type position_scene = screenshot_prefix.rfind("scenes/");
		if (position_scene != std::string::npos)
		  {
		    screenshot_prefix.replace(position_scene, 7, "share/screenshots/");
		  }

	      }
	    else
	      screenshot_prefix = "scene_";

	    //capture.setPrefix(ofilename.str());
	    capture.setPrefix(screenshot_prefix);

	    groot = scene;

	    createScene();

	    if (!keepParams) resetView();
				
	    update();
	  }


	  /// Render Scene called during multiThread simulation using automate
	  void QtOgreViewer::drawFromAutomate()
	  {
	    update();
	  }

	  void QtOgreViewer::automateDisplayVM(void)
	  {
	    std::vector<core::VisualModel *>::iterator it = simulation::automatescheduler::ThreadSimulation::getInstance()->vmodels.begin();
	    std::vector<core::VisualModel *>::iterator itEnd = simulation::automatescheduler::ThreadSimulation::getInstance()->vmodels.end();

	    while (it != itEnd)
	      {
		(*it)->draw();
		++it;
	      }
	  }


	  bool QtOgreViewer::updateInteractor(QMouseEvent * e)
	  {
	    if(e->state()&Qt::ShiftButton)
	      {	       
		SofaViewer::mouseEvent(e);
		return true;
	      }
	    return false;
	  }

	  void QtOgreViewer::moveRayPickInteractor(int eventX, int eventY)
	  {
	    sofa::defaulttype::Vector3  p0, px, py, pz;
	    GLint viewPort[4] = {0,0,width(), height()};
	    Ogre::Matrix4 modelViewMatrix = mCamera->getViewMatrix().transpose();
	    Ogre::Matrix4 projectionMatrix = mCamera->getProjectionMatrix(); 

	    double modelViewMatrixGL[16] =
	      {
		modelViewMatrix[0][0], modelViewMatrix[0][1], modelViewMatrix[0][2], modelViewMatrix[0][3],
		modelViewMatrix[1][0], modelViewMatrix[1][1], modelViewMatrix[1][2], modelViewMatrix[1][3],
		modelViewMatrix[2][0], modelViewMatrix[2][1], modelViewMatrix[2][2], modelViewMatrix[2][3],
		modelViewMatrix[3][0], modelViewMatrix[3][1], modelViewMatrix[3][2], modelViewMatrix[3][3],
	      };

	    double projectionMatrixGL[16] =
	      {
		projectionMatrix[0][0], projectionMatrix[0][1], projectionMatrix[0][2], projectionMatrix[0][3],
		projectionMatrix[1][0], projectionMatrix[1][1], projectionMatrix[1][2], projectionMatrix[1][3],
		projectionMatrix[2][0], projectionMatrix[2][1], projectionMatrix[2][2], -1.0,
		projectionMatrix[3][0], projectionMatrix[3][1], projectionMatrix[3][2], projectionMatrix[3][3],
	      };


	    gluUnProject(eventX, viewPort[3]-1-(eventY),  0, modelViewMatrixGL, projectionMatrixGL, viewPort, &(p0[0]), &(p0[1]), &(p0[2]));
	    gluUnProject(eventX+1, viewPort[3]-1-(eventY),0, modelViewMatrixGL, projectionMatrixGL, viewPort, &(px[0]), &(px[1]), &(px[2]));
	    gluUnProject(eventX, viewPort[3]-1-(eventY+1),0, modelViewMatrixGL, projectionMatrixGL, viewPort, &(py[0]), &(py[1]), &(py[2]));
	    gluUnProject(eventX, viewPort[3]-1-(eventY),  1, modelViewMatrixGL, projectionMatrixGL, viewPort, &(pz[0]), &(pz[1]), &(pz[2]));

	    pz*=-1;
	    px -= p0;
	    py -= p0;
	    pz -= p0;

	    px.normalize();
	    py.normalize();
	    pz.normalize();
	    sofa::defaulttype::Mat4x4d transform;
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


	    sofa::defaulttype::Mat3x3d mat; mat = transform;
	    sofa::defaulttype::Quat q; q.fromMatrix(mat);
	    //std::cout << p0[0]<<' '<<p0[1]<<' '<<p0[2] << " -> " << pz[0]<<' '<<pz[1]<<' '<<pz[2] << std::endl;
	    interactor->newPosition(p0, q, transform);
	  }

	  QString QtOgreViewer::helpString() 
	  {

	    QString text("<H1>QtOgreViewer</H1><br>\
<ul>\
<li><b>Mouse</b>: TO NAVIGATE<br></li>\
<li><b>Shift & Left Button</b>: TO PICK OBJECTS<br></li>\
<li><b>C</b>: TO CENTER THE VIEW<br></li>\
<li><b>B</b>: TO CHANGE THE BACKGROUND<br></li>\
<li><b>C</b>: TO SWITCH INTERACTION MODE: press the KEY C.<br>\
Allow or not the navigation with the mouse.<br></li>\
<li><b>T</b>: TO CHANGE BETWEEN A PERSPECTIVE OR AN ORTHOGRAPHIC CAMERA<br></li>\
<li><b>L</b>: TO DRAW SHADOWS<br></li>\
<li><b>R</b>: TO CHANGE THE RENDER MODE<br></li>\
<li><b>I</b>: TO SAVE A SCREENSHOT<br>\
The captured images are saved in the running project directory under the name format capturexxxx.bmp<br></li>\
<li><b>V</b>: TO SAVE A VIDEO<br>\
Each time the frame is updated a screenshot is saved<br></li>\
<li><b>Esc</b>: TO QUIT ::sofa:: <br></li></ul>");

	    return text;
	  }

	} //qtogre

      } //viewer

    } //qt

  } //gui

} //sofa
