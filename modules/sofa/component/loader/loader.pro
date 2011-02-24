SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentloader

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_LOADER

HEADERS += initLoader.h \
           MeshGmshLoader.h \
           MeshObjLoader.h \
           MeshOffLoader.h \
           MeshTrianLoader.h \
           MeshVTKLoader.h \
           MeshSTLLoader.h \
           MeshXspLoader.h \
	   OffSequenceLoader.h


SOURCES += initLoader.cpp \
           MeshGmshLoader.cpp \
           MeshObjLoader.cpp \
           MeshOffLoader.cpp \
           MeshTrianLoader.cpp \
           MeshVTKLoader.cpp \
           MeshSTLLoader.cpp \
           MeshXspLoader.cpp \
	   OffSequenceLoader.cpp

           


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
#LIBS += -lsofacomponentbase$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(loader-local.cfg): include(loader-local.cfg) 
