#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <stdio.h>

#include <sofa/simulation/tree/Simulation.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/BackTrace.h>

#if defined(SOFA_GPU_CUDA)
#include <sofa/gpu/cuda/mycuda.h>
using namespace sofa::gpu::cuda;
#endif

#include "ExampleApplication.h"
#include "DotSceneLoader.h"
#include "OgreVisualModel.h"

using namespace sofa::simulation::tree;

// Listener class for frame updates
class SofaListener : public ExampleFrameListener
{
protected:
    GNode* groot;
    Ogre::ShadowTechnique shadow;
    int mVidFrame;
public:
    SofaListener(RenderWindow* win, Camera* cam, GNode* groot)
    : ExampleFrameListener(win, cam), groot(groot), shadow(Ogre::SHADOWTYPE_NONE), mVidFrame(0)
    {
    }

    bool frameStarted(const FrameEvent& evt)
    {
        if( ExampleFrameListener::frameStarted(evt) == false )
            return false;
        //double dt = evt.timeSinceLastFrame;
        if (groot)
            Simulation::animate(groot);
        return true;
    }

    virtual bool processUnbufferedKeyInput(const FrameEvent& evt)
    {
        using namespace OIS;

        if( mKeyboard->isKeyDown(KC_L) && mTimeUntilNextToggle <= 0 )
        {
            switch(shadow)
            {
	    case Ogre::SHADOWTYPE_NONE:
		shadow = Ogre::SHADOWTYPE_STENCIL_ADDITIVE;
		break;
	    case Ogre::SHADOWTYPE_STENCIL_ADDITIVE:
	    default:
		shadow = Ogre::SHADOWTYPE_NONE;
		break;
            }
            mCamera->getSceneManager()->setShadowTechnique(shadow);
            mTimeUntilNextToggle = 0.5;
        }
        if( mKeyboard->isKeyDown(KC_V) )
        {
            std::ostringstream ss;
            ss << "capture" << mVidFrame/1000 << ((mVidFrame/100)%10) << ((mVidFrame/10)%10) << ((mVidFrame)%10) << ".png";
            mVidFrame++;
            mWindow->writeContentsToFile(ss.str());
        }
        return this->ExampleFrameListener::processUnbufferedKeyInput(evt);
    }
};

/** Application class */
class SofaApplication : public ExampleApplication
{
public:
    GNode* groot;
    std::string sofaFileName;
    std::string sceneName;

    SofaApplication()
    : groot(NULL)
    {
    }

protected:
    virtual void createAll(void)
    {
        // Create any resource listeners (for loading screens)
        createResourceListener();
        // Load resources
        loadResources();
        
        createScene();
        
        createFrameListener();
    }
    virtual void createScene(void)
    {
        //mSceneMgr = mRoot->createSceneManager(ST_GENERIC, "ExampleSMInstance");
        if (!sofaFileName.empty())
            groot = Simulation::load(sofaFileName.c_str());

        if (groot==NULL)
            groot = new GNode;

        DotSceneLoader loader;
        loader.parseDotScene(sceneName, "General", NULL); //mSceneMgr);
        mSceneMgr = loader.getSceneManager();
        
        if (groot)
        {
            // This could be done as an action, but it is shorter to do it this way
            vector<OgreVisualModel*> visualModels;
            groot->getTreeObjects<OgreVisualModel>(&visualModels);
            for (unsigned int i=0; i<visualModels.size(); i++)
                visualModels[i]->attach(mSceneMgr);
        }
        
        
        //createCamera();
        Ogre::SceneManager::CameraIterator itCam = mSceneMgr->getCameraIterator();
        if (itCam.hasMoreElements())
            mCamera = itCam.getNext();
        else
        {
            // Create the camera
            mCamera = mSceneMgr->createCamera("DefaultCam");
            // Position it at 50 in Z direction
            mCamera->setPosition(Vector3(0,10,50));
            // Look back along -Z
            mCamera->lookAt(Vector3(0,0,0));
            mCamera->setNearClipDistance(loader.environment.nearClipDistance);
            mCamera->setFarClipDistance(loader.environment.farClipDistance);
        }
        
        //createViewports();
        // Create one viewport, entire window
        Viewport* vp = mWindow->addViewport(mCamera);
        vp->setBackgroundColour(loader.environment.backgroundColour);
        //ColourValue bgColour( 56/255.0, 56/255.0, 70/255.0 );
        //vp->setBackgroundColour( bgColour );

        // Alter the camera aspect ratio to match the viewport
        mCamera->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
    }

    void createFrameListener(void)
    {
        // This is where we instantiate our own frame listener
        mFrameListener= new SofaListener(mWindow, mCamera, groot);
        mRoot->addFrameListener(mFrameListener);
    }
};

// ---------------------------------------------------------------------
// --- MAIN
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    // Replace OpenGL visual models with OgreVisualModel
    sofa::core::ObjectFactory::AddAlias("OglModel","OgreVisualModel",true);
    sofa::core::ObjectFactory::AddAlias("VisualModel","OgreVisualModel",true);


    std::string fileName = "../scenes/ogreChainFFD.scn";
    std::string sceneName = "../scenes/Ogre3D/default.scene";
    if (argc < 2 || argc > 3)
    {
        std::cerr << "Usage: "<<argv[0]<<" filename.scn [ogrefile.scene]\n";
        //return 1;
    }
    else
    {
        fileName = argv[1];
        if (argc >=3)
            sceneName = argv[2];
        else
        {
            // find the scene with the same base name
            sceneName = fileName.substr(0,fileName.rfind("."))+".scene";
            std::ifstream in(sceneName.c_str());
            if (in.fail())
            {
                std::cout << "File "<<sceneName<<" not found."<<std::endl;
                sceneName = "default.scene";
            }
            std::cout << "Using scene file "<<sceneName<<std::endl;
        }
    }

#if defined(SOFA_GPU_CUDA)
    mycudaInit(0);
#endif

    SofaApplication app;
    app.sofaFileName = fileName;
    app.sceneName = sceneName;

    try {
        app.go();
    } catch( Ogre::Exception& e ) {
        std::cerr << "An exception has occured: " << e.getFullDescription();
    }

    return 0;
}
