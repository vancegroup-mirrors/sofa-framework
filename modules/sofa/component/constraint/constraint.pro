SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentconstraint

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_CONSTRAINT

HEADERS +=  initConstraint.h \
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
           OscillatorConstraint.h \
           OscillatorConstraint.inl \
           LinearSolverConstraintCorrection.h \
           LinearSolverConstraintCorrection.inl \
           ParabolicConstraint.h \
           ParabolicConstraint.inl \
           PrecomputedConstraintCorrection.h \
           PrecomputedConstraintCorrection.inl \
           UncoupledConstraintCorrection.h \
           UncoupledConstraintCorrection.inl \
           UnilateralInteractionConstraint.h \
           UnilateralInteractionConstraint.inl \
           LinearMovementConstraint.h \
           LinearMovementConstraint.inl \
           LinearVelocityConstraint.h \
           LinearVelocityConstraint.inl

SOURCES += initConstraint.cpp \
           AttachConstraint.cpp \
           FixedConstraint.cpp \
           FixedPlaneConstraint.cpp \
           FixedRotationConstraint.cpp \
           FixedTranslationConstraint.cpp \
           HermiteSplineConstraint.cpp \
           OscillatorConstraint.cpp \
           LinearSolverConstraintCorrection.cpp \
           ParabolicConstraint.cpp \
           PrecomputedConstraintCorrection.cpp \
           UncoupledConstraintCorrection.cpp \
           UnilateralInteractionConstraint.cpp \
           LinearMovementConstraint.cpp  \
           LinearVelocityConstraint.cpp 

contains(DEFINES,SOFA_HAVE_EIGEN2){ 
HEADERS += \
           DistanceConstraint.h \
           DistanceConstraint.inl \
           FixedLMConstraint.h \
           FixedLMConstraint.inl \
           RotationLMConstraint.h \
           RotationLMConstraint.inl 
           
SOURCES += \           
           DistanceConstraint.cpp \
           FixedLMConstraint.cpp \
           RotationLMConstraint.cpp
}




LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentmass$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(component-local.cfg) 
