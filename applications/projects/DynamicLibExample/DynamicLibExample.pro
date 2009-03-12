SOFA_DIR=../../..
TEMPLATE = lib

include($${SOFA_DIR}/sofa.cfg)

TARGET = DynamicLibExample$$LIBSUFFIX

CONFIG += $$CONFIGLIBRARIES
CONFIG -= staticlib
CONFIG += dll
DEFINES += SOFA_BUILD_DYNAMICLIBEXAMPLE

LIBS += $$SOFA_LIBS
LIBS += $$SOFA_EXT_LIBS
INCLUDEPATH += $$SOFA_DIR/extlibs

SOURCES = MyFakeComponent.cpp \
          OtherFakeComponent.cpp \
	    initDynamicLibExample.cpp

HEADERS = MyFakeComponent.h \
          OtherFakeComponent.h 




 


