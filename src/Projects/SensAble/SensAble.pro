include(../../../sofa.cfg)
TEMPLATE = app
CONFIG += $$CONFIGPROJECT \
          warn_on

DESTDIR = ../../../bin
TARGET = phantom$$SUFFIX

OBJECTS_DIR = OBJ/$$CONFIGDEBUG
INCLUDEPATH = ../..
INCLUDEPATH += ../../../include
DEPENDPATH = ../..
SOURCES = Main.cpp SensAble.cpp
HEADERS = SensAble.h

contains (DEFINES, SOFA_GUI_QT) {
CONFIG += qt
QT += opengl qt3support
}

QMAKE_LIBDIR = ../../../lib/$$LIBSDIRECTORY ../../../lib/$$LIBSDIRECTORY/../Common
LIBS = -lSofaAbstract$$LIBSUFFIX -lSofaCore$$LIBSUFFIX -lSofaComponents$$LIBSUFFIX -lNewMat$$LIBSUFFIX

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

# add the open haptics include path and library path
win32{
  3DTOUCH_BASE = C:\Program Files\SensAble\3DTouch
  INCLUDEPATH += "$$3DTOUCH_BASE/include"
  INCLUDEPATH += "$$3DTOUCH_BASE/utilities/include"
  LIBS += -lhd -lhdu -lhl
  QMAKE_LIBDIR += "$$3DTOUCH_BASE/lib"
  QMAKE_LIBDIR += "$$3DTOUCH_BASE/utilities/lib"
}
unix{
  INCLUDEPATH += $(3DTOUCH_BASE)/include
  INCLUDEPATH += $(3DTOUCH_BASE)/utilities/include
  LIBS += -lhd -lhdu -lhl
  QMAKE_LIBDIR += $(3DTOUCH_BASE)/lib
  QMAKE_LIBDIR += $(3DTOUCH_BASE)/utilities/lib
}
