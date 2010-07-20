SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentinteractionforcefield

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_INTERACTIONFORCEFIELD

HEADERS += \
          RepulsiveSpringForceField.h \
          RepulsiveSpringForceField.inl \
          InteractionEllipsoidForceField.h \
          InteractionEllipsoidForceField.inl
           
SOURCES += initInteractionForceField.cpp \
          RepulsiveSpringForceField.cpp \
          InteractionEllipsoidForceField.cpp



LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(interactionforcefield-local.cfg) 
