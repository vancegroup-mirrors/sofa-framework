SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentbehaviormodel

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_BEHAVIORMODEL

HEADERS += \
           eulerianfluid/Fluid2D.h \
           eulerianfluid/Fluid3D.h \
           eulerianfluid/Grid2D.h \
           eulerianfluid/Grid3D.h \
#	   elasticmembrane/cfortran.h \
#	   elasticmembrane/FluidMembrane.h  \
#	   elasticmembrane/Levelset.h \
#	   elasticmembrane/MACGrid.h \
#          elasticmembrane/MeshLevelSetGenerator.h \
#	   elasticmembrane/parser.h

SOURCES += initBehaviorModel.cpp \
           eulerianfluid/Fluid2D.cpp \
           eulerianfluid/Fluid3D.cpp \
           eulerianfluid/Grid2D.cpp \
           eulerianfluid/Grid3D.cpp \
#	   elasticmembrane/FluidMembrane.cpp \ 
#	   elasticmembrane/Levelset.cpp \
#	   elasticmembrane/MACGrid.cpp \
#	   elasticmembrane/MeshLevelSetGenerator.cpp 

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
contains(DEFINES,SOFA_HAVE_FISHPACK){
contains(DEFINES, MUPARSER){
LIBS += -lfftpack -lfishpack -lfftpack -lmuParser
}
}

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(behaviormodel-local.cfg) 
