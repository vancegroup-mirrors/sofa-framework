#		    l i b Q G L V i e w e r
#	C o m p i l a t i o n    c o n f i g u r a t i o n

# Run "qmake; make; make install" to compile and install the library.
# Optional arguments can tune install paths (as in "qmake PREFIX=$HOME"). See doc/download.html for details.

# Attention : Windows Qt 2.3 users should use QGLViewer.Qt2.3.pro instead of this file.

# If your Qt version is lower than 3.1 (look at $QTDIR/lib), you need to link with GLUT.
# Uncomment the following line:
# USE_GLUT = yes

VERSION = 2.3.0

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET  = QGLViewer$${LIBSUFFIX}
CONFIG -= debug
CONFIG *= release qt opengl warn_on thread create_prl uic
DEFINES *= QT3_SUPPORT

win32 {

  CONFIG *= dll
  DEFINES *= CREATE_QGLVIEWER_DLL

  # Use the DLL version of Qt
  DEFINES *= QT_DLL QT_THREAD_SUPPORT

  # Required to use dynamic_cast
  CONFIG *= rtti

  # Make sure to have C++ files, PentiumPro code, few warnings, add
  # support to RTTI and Exceptions, and generate debug info "program database"
  # Any feedback on these flags is welcome.
  !win32-g++ {
    QMAKE_CXXFLAGS *= -TP -G6 -GR -Zi
    win32-msvc {
      QMAKE_CXXFLAGS *= -GX
    } else {
      QMAKE_CXXFLAGS *= -EHs
    }
  }

}



MOC_DIR           = moc
OBJECTS_DIR       = obj

HEADERS = qglviewer.h \
	  camera.h \
	  manipulatedFrame.h \
	  manipulatedCameraFrame.h \
	  frame.h \
	  constraint.h \
	  keyFrameInterpolator.h \
	  mouseGrabber.h \
	  quaternion.h \
	  vec.h \
	  domUtils.h \
	  config.h

SOURCES = qglviewer.cpp \
	  camera.cpp \
	  manipulatedFrame.cpp \
	  manipulatedCameraFrame.cpp \
	  frame.cpp \
	  saveSnapshot.cpp \
	  constraint.cpp \
	  keyFrameInterpolator.cpp \
	  mouseGrabber.cpp \
	  quaternion.cpp \
	  vec.cpp

DISTFILES *= qglviewer-icon.xpm

#TRANSLATIONS = qglviewer_fr.ts
                       
QT_VERSION=$$[QT_VERSION]

contains( QT_VERSION, "^4.*" ) {
  QT *= sql xml network opengl
}

#		--  I m a g e I n t e r f a c e  --
contains( QT_VERSION, "^4.*" ) {
  FORMS *= ImageInterface.Qt4.ui
} else {
  FORMS *= ImageInterface.Qt3.ui
}



#		--  V e c t o r i a l   R e n d e r i n g  --
# In case of compilation troubles with vectorial rendering, uncomment this line
# DEFINES *= NO_VECTORIAL_RENDER

contains( DEFINES, NO_VECTORIAL_RENDER ) {
  message( Vectorial rendering disabled )
} else {
  contains( QT_VERSION, "^4.*" ) {
    FORMS *= VRenderInterface.Qt4.ui
  } else {
    FORMS *= VRenderInterface.Qt3.ui
  }

  SOURCES *= \
	VRender/BackFaceCullingOptimizer.cpp \
	VRender/BSPSortMethod.cpp \
	VRender/EPSExporter.cpp \
	VRender/Exporter.cpp \
	VRender/FIGExporter.cpp \
	VRender/gpc.cpp \
	VRender/ParserGL.cpp \
	VRender/Primitive.cpp \
	VRender/PrimitivePositioning.cpp \
	VRender/TopologicalSortMethod.cpp \
	VRender/VisibilityOptimizer.cpp \
	VRender/Vector2.cpp \
	VRender/Vector3.cpp \
	VRender/NVector3.cpp \
	VRender/VRender.cpp

  HEADERS *= \
	VRender/AxisAlignedBox.h \
	VRender/Exporter.h \
	VRender/gpc.h \
	VRender/NVector3.h \
	VRender/Optimizer.h \
	VRender/ParserGL.h \
	VRender/Primitive.h \
	VRender/PrimitivePositioning.h \
	VRender/SortMethod.h \
	VRender/Types.h \
	VRender/Vector2.h \
	VRender/Vector3.h \
	VRender/VRender.h
}
