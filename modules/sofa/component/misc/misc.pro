SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentmisc

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_MISC

HEADERS += initMisc.h \
           CompareState.h \
           CompareTopology.h \
           DevAngleCollisionMonitor.h \
           DevAngleCollisionMonitor.inl \
           DevTensionMonitor.h \
           DevTensionMonitor.inl \
           DevMonitorManager.h \
           ExtraMonitor.h \
           ExtraMonitor.inl \
           InputEventReader.h \
           Monitor.h \
           Monitor.inl \
           ParticleSink.h \
           ParticleSource.h \
           ReadState.h \
           ReadState.inl \
           ReadTopology.h \
           ReadTopology.inl \
           WriteState.h \
           WriteState.inl \
           WriteTopology.h \
           WriteTopology.inl \
           EvalPointsDistance.h \
           EvalPointsDistance.inl \
           EvalSurfaceDistance.h \
           EvalSurfaceDistance.inl \
           MeshTetraStuffing.h \
           TopologicalChangeProcessor.h


SOURCES += initMisc.cpp \
           CompareState.cpp \
           CompareTopology.cpp \
           DevAngleCollisionMonitor.cpp \
           DevTensionMonitor.cpp \
           DevMonitorManager.cpp \
           ExtraMonitor.cpp \
           InputEventReader.cpp \
           Monitor.cpp \
           ParticleSink.cpp \
           ParticleSource.cpp \
           ReadState.cpp \
           ReadTopology.cpp \
           WriteState.cpp \
           WriteTopology.cpp \
           EvalPointsDistance.cpp \
           EvalSurfaceDistance.cpp \
           MeshTetraStuffing.cpp \
           TopologicalChangeProcessor.cpp



LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentmastersolver$$LIBSUFFIX
LIBS += -lsofacomponentfem$$LIBSUFFIX
LIBS += -lsofacomponentinteractionforcefield$$LIBSUFFIX
LIBS += -lsofacomponentcontextobject$$LIBSUFFIX
LIBS += -lsofacomponentbehaviormodel$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentcontroller$$LIBSUFFIX
LIBS += -lsofacomponentvisualmodel$$LIBSUFFIX
LIBS += -lsofacomponentmass$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentmapping$$LIBSUFFIX
LIBS += -lsofacomponentconstraint$$LIBSUFFIX
LIBS += -lsofacomponentcollision$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(misc-local.cfg) 
