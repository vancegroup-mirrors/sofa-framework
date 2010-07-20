SOFA_DIR = ../../../..
TEMPLATE = lib

TARGET = sofacomponentfem

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}


DEFINES += SOFA_BUILD_COMPONENT_FEM
#DEFINES += SOFA_BUILD_COMPONENT_FEM_FETYPE
#DEFINES += SOFA_BUILD_COMPONENT_FEM_MATERIAL
#DEFINES += SOFA_BUILD_COMPONENT_FEM_FORCEFIELD
#DEFINES += SOFA_BUILD_COMPONENT_FEM_STRAINTENSOR

HEADERS += initFEM.h

SOURCES += initFEM.cpp



LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(fem-local.cfg) 
