SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentprojectiveconstraintset

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_PROJECTIVECONSTRAINTSET

HEADERS +=  initProjectiveConstraintSet.h \
           AttachConstraint.h \
           AttachConstraint.inl \
           FixedConstraint.h \
           FixedConstraint.inl \  
           FixedPlaneConstraint.h \
           FixedPlaneConstraint.inl \
           FixedRotationConstraint.h \
           FixedRotationConstraint.inl \
           FixedTranslationConstraint.h \
           FixedTranslationConstraint.inl \
           HermiteSplineConstraint.h \
           HermiteSplineConstraint.inl \
           LinearMovementConstraint.h \
           LinearMovementConstraint.inl \
           LinearVelocityConstraint.h \
           LinearVelocityConstraint.inl \
           OscillatorConstraint.h \
           OscillatorConstraint.inl \
           ParabolicConstraint.h \
           ParabolicConstraint.inl \
           PartialFixedConstraint.h \
           PartialFixedConstraint.inl \
#          PartialLinearMovementConstraint.h \
#          PartialLinearMovementConstraint.inl

SOURCES += initProjectiveConstraintSet.cpp \
           AttachConstraint.cpp \
           FixedConstraint.cpp \
           FixedPlaneConstraint.cpp \
           FixedRotationConstraint.cpp \
           FixedTranslationConstraint.cpp \
           HermiteSplineConstraint.cpp \
           LinearMovementConstraint.cpp \
           LinearVelocityConstraint.cpp \
           OscillatorConstraint.cpp \
           ParabolicConstraint.cpp \
           PartialFixedConstraint.cpp \
#          PartialLinearMovementConstraint.cpp




LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += -lsofacomponentmass$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(component-local.cfg): include(component-local.cfg) 
