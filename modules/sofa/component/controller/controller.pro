SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentcontroller$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_CONTROLLER

HEADERS += initController.h \
           ArticulatedHierarchyController.h \
           ArticulatedHierarchyBVHController.h \
           Controller.h \
           EdgeSetController.h \
           EdgeSetController.inl \
           MechanicalStateController.h \
           MechanicalStateController.inl

SOURCES += initController.cpp \
           ArticulatedHierarchyController.cpp \
           ArticulatedHierarchyBVHController.cpp \
           Controller.cpp \
           EdgeSetController.cpp \
           MechanicalStateController.cpp



contains(DEFINES,SOFA_HAVE_SENSABLE){

    HEADERS +=  OmniDriver.h \
           	    ForceFeedback.h \
           	    NullForceFeedback.h \
           	    EnslavementForceFeedback.h \
		        LCPForceFeedback.h

    SOURCES +=  OmniDriver.cpp \
           	    NullForceFeedback.cpp \
           	    EnslavementForceFeedback.cpp \
		        LCPForceFeedback.cpp

}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentmastersolver$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(controller-local.cfg) 
