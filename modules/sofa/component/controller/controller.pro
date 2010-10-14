SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentcontroller

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_CONTROLLER

HEADERS += initController.h \
           ArticulatedHierarchyController.h \
           ArticulatedHierarchyBVHController.h \
           Controller.h \
           EdgeSetController.h \
           EdgeSetController.inl \
           MechanicalStateController.h \
           MechanicalStateController.inl \
           ForceFeedback.h \
           NullForceFeedback.h \
           EnslavementForceFeedback.h \
           LCPForceFeedback.h \
           LCPForceFeedback.inl 

SOURCES += initController.cpp \
           ArticulatedHierarchyController.cpp \
           ArticulatedHierarchyBVHController.cpp \
           Controller.cpp \
           EdgeSetController.cpp \
           MechanicalStateController.cpp \
           NullForceFeedback.cpp \
           EnslavementForceFeedback.cpp \
           LCPForceFeedback.cpp 

contains(DEFINES,SOFA_HAVE_SENSABLE){
    HEADERS +=  OmniDriver.h
    SOURCES +=  OmniDriver.cpp
}


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentprojectiveconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentinteractionforcefield$$LIBSUFFIX
LIBS += -lsofacomponentmastersolver$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(controller-local.cfg) 
