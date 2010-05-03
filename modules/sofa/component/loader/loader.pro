SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentloader$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_LOADER

HEADERS += initLoader.h \
           MeshGmshLoader.h \
           MeshObjLoader.h \
           MeshTrianLoader.h \
           MeshVTKLoader.h \
           MeshXspLoader.h 


SOURCES += initLoader.cpp \
           MeshGmshLoader.cpp \
           MeshObjLoader.cpp \
           MeshTrianLoader.cpp \
           MeshVTKLoader.cpp \
           MeshXspLoader.cpp 

           


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
#LIBS += -lsofacomponentbase$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(loader-local.cfg) 
