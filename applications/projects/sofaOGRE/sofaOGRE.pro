SOFA_DIR=../../..
TEMPLATE = app

include($${SOFA_DIR}/sofa.cfg)

TARGET = sofaOGRE$$SUFFIX
DESTDIR = $$SOFA_DIR/bin
CONFIG += $$CONFIGPROJECT
LIBS += $$SOFA_LIBS

SOURCES = DotSceneLoader.cpp \
          OgreVisualModel.cpp \
          Main.cpp \
          tinyxml.cpp \
          tinyxmlerror.cpp \
          tinyxmlparser.cpp

HEADERS = DotSceneLoader.h \
          OgreVisualModel.h \
          ExampleApplication.h \
          ExampleFrameListener.h \
          tinyxml.h

RC_FILE = sofa.rc

########################################################################
#  OGRE 3D
########################################################################

win32 {
	INCLUDEPATH += $(OGRE_HOME)/include
	QMAKE_LIBDIR += $(OGRE_HOME)/lib
	LIBS += OgreMain.lib OIS.lib
}

unix {
	QMAKE_CXXFLAGS += $$system(pkg-config --cflags OGRE OIS)
	LIBS += $$system(pkg-config --libs OGRE OIS)
	#CONFIG += link_pkgconfig
	#PKGCONFIG += OGRE
	#PKGCONFIG += CEGUI
	#PKGCONFIG += OIS
}

########################################################################
