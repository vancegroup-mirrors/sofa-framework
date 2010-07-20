SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentconstraintset

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_CONSTRAINTSET

HEADERS +=  initConstraintSet.h \
           LCPConstraintSolver.h \
           LinearSolverConstraintCorrection.h \
           LinearSolverConstraintCorrection.inl \
           PrecomputedConstraintCorrection.h \
           PrecomputedConstraintCorrection.inl \
           UncoupledConstraintCorrection.h \
           UncoupledConstraintCorrection.inl \
           UnilateralInteractionConstraint.h \
           UnilateralInteractionConstraint.inl 

SOURCES += initConstraintSet.cpp \
           LCPConstraintSolver.cpp \
           LinearSolverConstraintCorrection.cpp \
           PrecomputedConstraintCorrection.cpp \
           UncoupledConstraintCorrection.cpp \
           UnilateralInteractionConstraint.cpp 

contains(DEFINES,SOFA_HAVE_EIGEN2){ 
HEADERS += \
           LMConstraintSolver.h\
           DistanceLMConstraint.h \
           DistanceLMConstraint.inl \
           FixedLMConstraint.h \
           FixedLMConstraint.inl \
           DOFBlockerLMConstraint.h \
           DOFBlockerLMConstraint.inl 
           
SOURCES += \           
           LMConstraintSolver.cpp\
           DistanceLMConstraint.cpp \
           FixedLMConstraint.cpp \
           DOFBlockerLMConstraint.cpp
}




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

include(component-local.cfg) 
