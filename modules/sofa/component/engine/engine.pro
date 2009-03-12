SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentengine$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_ENGINE

HEADERS += initEngine.h \
           BoxROI.h \
           BoxROI.inl

SOURCES += initEngine.cpp \
           BoxROI.cpp

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(engine-local.cfg) 
