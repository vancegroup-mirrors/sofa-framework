SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentbehaviormodel$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_BEHAVIORMODEL

HEADERS += \
           eulerianfluid/Fluid2D.h \
           eulerianfluid/Fluid3D.h \
           eulerianfluid/Grid2D.h \
           eulerianfluid/Grid3D.h 

SOURCES += initBehaviorModel.cpp \
           eulerianfluid/Fluid2D.cpp \
           eulerianfluid/Fluid3D.cpp \
           eulerianfluid/Grid2D.cpp \
           eulerianfluid/Grid3D.cpp 

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(behaviormodel-local.cfg) 
