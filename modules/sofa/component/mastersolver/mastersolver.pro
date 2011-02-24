SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentmastersolver

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_MASTERSOLVER

HEADERS += initMasterSolver.h \
           DefaultMasterSolver.h \
           MultiStepMasterSolver.h \
           MultiTagMasterSolver.h \
           FreeMotionMasterSolver.h 

SOURCES += initMasterSolver.cpp \
           DefaultMasterSolver.cpp \
           MultiStepMasterSolver.cpp \
           MultiTagMasterSolver.cpp \
           FreeMotionMasterSolver.cpp



LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentprojectiveconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(mastersolver-local.cfg) 
