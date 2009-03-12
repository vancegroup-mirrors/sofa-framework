SOFA_DIR=../../../..
TEMPLATE = app

include($${SOFA_DIR}/sofa.cfg)

EXEC = Haptic
TARGET = Haptic$$SUFFIX
DESTDIR = $$SOFA_DIR/bin
CONFIG += $$CONFIGPROJECTGUI
LIBS += $$SOFA_GUI_LIBS
LIBS += $$SOFA_LIBS
LIBS += -lchaiIntegration$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS
CONFIG += $$CONFIGLIBRARIES

INCLUDEPATH += $$SOFA_EXT_LIBS


# The following is a workaround to get KDevelop to detect the name of the program to start
unix {
	QMAKE_POST_LINK = ln -sf $$TARGET $$DESTDIR/$$EXEC-latest
}


SOURCES = Main.cpp 
HEADERS = 


# The folowing is the link with chai3D library and with SensAble 3Dtouch library
win32{
  	
  	QMAKE_LIBDIR += "$$3DTOUCH_BASE/lib"
  	QMAKE_LIBDIR += "$$3DTOUCH_BASE/utilities/lib"
	contains (CONFIGDEBUG, debug) {
 		LIBS += -lchai3d_completed
	}
	contains (CONFIGDEBUG, release) {
		LIBS += -lchai3d_complete
	} 
}
unix{
	QMAKE_LIBDIR += "/usr/lib"
	LIBS += -lchai3d_linux -lHD -HDU
	DEFINES += _POSIX _MAX_PATH=260 _LINUX LINUX
}

