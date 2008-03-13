include(../../../sofa.cfg)
TEMPLATE = app
CONFIG += $$CONFIGPROJECT \
          warn_on

DESTDIR = ../../../bin
TARGET = SofaKaapi$$SUFFIX

OBJECTS_DIR = OBJ/$$CONFIGDEBUG
INCLUDEPATH = ../..
INCLUDEPATH += ../../../include
DEPENDPATH = ../..

SOURCES = Main.cpp 
HEADERS = 

contains (DEFINES, SOFA_GUI_QT) {
CONFIG += qt
QT += opengl qt3support
}

QMAKE_LIBDIR = ../../../lib/$$LIBSDIRECTORY ../../../lib/$$LIBSDIRECTORY/../Common
LIBS = -lSofaAbstract$$LIBSUFFIX -lSofaCore$$LIBSUFFIX -lSofaComponents$$LIBSUFFIX -lNewMat$$LIBSUFFIX

########################################################################
#  KAAPI
########################################################################
KAAPI = ../../../../KaapiSVN

INCLUDEPATH += $$KAAPI $$KAAPI/utils $$KAAPI/kernel $$KAAPI/rfo $$KAAPI/dfg $$KAAPI/network $$KAAPI/network/nameserver $$KAAPI/network\stat $$KAAPI/st $$KAAPI/ws $$KAAPI/ft
DEPENDPATH += $$KAAPI
LIBS += -lkaapi
win32{
	LIBS += -lpthreadVC2 -lWs2_32
    INCLUDEPATH += $$KAAPI/win32
	contains (CONFIGDEBUG, debug) {
		QMAKE_LIBDIR += $$KAAPI/win32 $$KAAPI/win32/Debug	
		QMAKE_CXXFLAGS_DEBUG += -wd4290 -wd4996 -wd4311 -wd4312
	}	
	contains (CONFIGDEBUG, release) {
		QMAKE_LIBDIR += $$KAAPI/win32 $$KAAPI/win32/Release
		QMAKE_CXXFLAGS_RELEASE += -wd4290 -wd4996 -wd4311 -wd4312
	}
}
else{
	QMAKE_LIBDIR += $$KAAPI/lib
}
contains (CONFIGDEBUG, debug) {
    DEFINES += KAAPI_DEBUG DEBUG VERBOSE_ENABLE
}
########################################################################

win32{
  LIBS += -llibxml2 -lGLaux -lglut32 -lcomctl32 -lopengl32 -lglu32 -lAdvAPI32 -lUser32 -lShell32 -lGdi32 -lWSock32 -lWS2_32 -lOle32
  contains (DEFINES, SOFA_GUI_FLTK) {
	LIBS += -lSofaGUIFLTK$$LIBSUFFIX -lfltk -lfltkgl
  }
  contains (DEFINES, SOFA_GUI_QT) {
	LIBS += -lSofaGUIQT$$LIBSUFFIX
  }
  contains (CONFIGPROJECT, vc7) {
	contains (CONFIGDEBUG, debug) {
	  	QMAKE_LFLAGS += /NODEFAULTLIB:libcd /NODEFAULTLIB:MSVCRT	
	}	
	contains (CONFIGDEBUG, release) {
	  	QMAKE_LFLAGS += /NODEFAULTLIB:libc /NODEFAULTLIB:MSVCRTD
	}
  }
  #QMAKE_LFLAGS = 
  #QMAKE_LIBS_WINDOWS = ""
  #QMAKE_CXXFLAGS += -GR -GX
  #DEFINES = WIN32
}

unix {
  LIBS += -L/usr/X11R6/lib -lglut -lGL -lGLU -lpthread -lxml2 -lz
  contains (DEFINES, SOFA_GUI_FLTK) {
	LIBS += -lSofaGUIFLTK$$LIBSUFFIX -lfltk_gl -lfltk
  }
  contains (DEFINES, SOFA_GUI_QT) {
	LIBS += -lSofaGUIQT$$LIBSUFFIX
  }
}
