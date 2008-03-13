# File generated by kdevelop's qmake manager. 
# ------------------------------------------- 
# Subdir relative project main directory: .
# Target is an application:  run$$SUFFIX

HEADERS += 
SOURCES += main.cpp 
include(../Projects.cfg)
TEMPLATE = app
CONFIG *= console warn_on qt opengl
DEFINES *= QT_CLEAN_NAMESPACE
OBJECTS_DIR = .obj
MOC_DIR = .moc
CONFIG += $$CONFIGPROJECT \
warn_on
DESTDIR = ../../../bin
TARGET = run$$SUFFIX
OBJECTS_DIR = OBJ/$$CONFIGDEBUG
INCLUDEPATH += ../../../include
DEPENDPATH = ../..
QMAKE_LIBDIR = ../../../lib/$$LIBSDIRECTORY ../../../lib/$$LIBSDIRECTORY/../Common
LIBS = -lSofaAbstract$$LIBSUFFIX -lSofaCore$$LIBSUFFIX -lSofaComponents$$LIBSUFFIX -lSofaCoin$$LIBSUFFIX 
contains (DEFINES, SOFA_GUI_QT){
  CONFIG += qt
  QT += opengl qt3support
}
win32{
  HEADERS *= qt4/MainGuiOiq.h \
  qt4/BuildQSceneGraphViewActionOiq.h \
  qt4/TreeItemFieldOiq.h \
  qt4/TreeItemNodeOiq.h
  SOURCES *= qt4/MainGuiOiq.cpp \
  qt4/BuildQSceneGraphViewActionOiq.cpp \
  qt4/TreeItemFieldOiq.cpp \
  qt4/TreeItemNodeOiq.cpp
  QMAKE_CXXFLAGS_DEBUG *= -DCOIN_DLL -DWIN32 -DSOWIN_DLL
  LIBS *= -lsimage1 -lsowin1 -lcoin2
  LIBS += -llibxml2 -lGLaux -lglut32 -lcomctl32 -lopengl32 -lglu32 -lAdvAPI32 -lUser32 -lShell32 -lGdi32 -lWSock32 -lWS2_32 -lOle32 -lNewMat
  contains (DEFINES, SOFA_GUI_FLTK){
    LIBS += -lSofaGUIFLTK$$LIBSUFFIX -lfltk -lfltkgl
  }
  contains (DEFINES, SOFA_GUI_QT){
    LIBS += -lSofaGUIQT$$LIBSUFFIX
  }
  contains (CONFIGPROJECT, vc7){
    contains (CONFIGDEBUG, debug){
    }
    contains (CONFIGDEBUG, release){
    }
  }
}
unix{
  CONFIG *= x11 opengl
  LIBS *= -L/usr/X11R6/lib -lglut -lGL -lGLU -lpthread -lxml2 -lz -lNewMat -lCoin -lSoQt -lSofaGUIQT$$LIBSUFFIX
}
