SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentcontextobject

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_CONTEXTOBJECT

HEADERS += \
           CoordinateSystem.h \
           Gravity.h 

SOURCES += initContextObject.cpp \
           CoordinateSystem.cpp \
           Gravity.cpp 


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(contextobject-local.cfg): include(contextobject-local.cfg) 
