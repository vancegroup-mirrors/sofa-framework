
#include "OgreVector3.h"
#include "viewer/qtogre/QtOgreViewer.h"
#include <GenGraphForm.h>

#ifdef QT_MODULE_QT3SUPPORT
#include <QFileDialog>
#else
#include <qfiledialog.h>
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

	  //*****************************************************************************************
	  // Just modify the point of view, keep the simulation running.
	  void QtOgreViewer::resetView()
	  {
	    using namespace Ogre;

	    //if no view file was present, we set up automatically the view.
	    showEntireScene();

	    if (!sceneFileName.empty())
	      {
		std::string viewFileName = sceneFileName+".ogre.view";
		std::ifstream in(viewFileName.c_str());
		if (!in.fail())
		  {
		    Ogre::Vector3 camera_position;
		    Ogre::Quaternion camera_orientation;

		    in >> camera_position[0];
		    in >> camera_position[1];
		    in >> camera_position[2];


		    in >> camera_orientation.w;
		    in >> camera_orientation.x;
		    in >> camera_orientation.y;
		    in >> camera_orientation.z;
		    camera_orientation.normalise();

		    mCamera->setOrientation(camera_orientation);			

		    mCamera->setPosition(camera_position);

		    in.close();
		    return;
		  }		
	      } 
	    update();
	  }



	  //*****************************************************************************************
	  //Screenshot


	  void QtOgreViewer::screenshot(const std::string filename)
	  {				
	    update();
	    mRenderWindow->writeContentsToFile(filename);						
	    std::cout << "Screen captured to " << filename << "\n"; 
	  }

	  //*****************************************************************************************
	  // Resize
	  void QtOgreViewer::setSizeW( int size )
	  {

	    if (mRenderWindow != NULL){
	      mRenderWindow->resize( size, mRenderWindow->getHeight());
	      mRenderWindow->windowMovedOrResized();
	    }

	    update();
	  }

	  void QtOgreViewer::setSizeH( int size )
	  {
	    if (mRenderWindow != NULL){
	      mRenderWindow->resize( mRenderWindow->getWidth(), size);
	      mRenderWindow->windowMovedOrResized();
	    }
	    update();
	  }


	  void QtOgreViewer::saveView()
	  {
	    if (!sceneFileName.empty())
	      {
		std::string viewFileName = sceneFileName+".ogre.view";
		std::ofstream out(viewFileName.c_str());
		if (!out.fail())
		  {
		    Ogre::Vector3 position_cam = mCamera->getPosition();
		    Ogre::Quaternion orientation_cam = mCamera->getOrientation();

		    out << position_cam[0] << " " << position_cam[1] << " " << position_cam[2] << "\n";
		    out << orientation_cam[0] << " " << orientation_cam[1] << " " << orientation_cam[2] << " " << orientation_cam[3] << "\n";
		    out.close();


		  }
		std::cout << "View parameters saved in "<<viewFileName<<std::endl;
	      }
	  }


	} //qtogre
      } //viewer
    } //qt
  } //gui
} //sofa
