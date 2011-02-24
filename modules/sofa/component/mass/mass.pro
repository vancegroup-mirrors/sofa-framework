SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentmass

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_MASS

HEADERS += initMass.h \
           AddMToMatrixFunctor.h \
           DiagonalMass.h \
           DiagonalMass.inl \
           MatrixMass.h \
           MatrixMass.inl \
           UniformMass.h \
           UniformMass.inl \
           MeshMatrixMass.h \
           MeshMatrixMass.inl

SOURCES += initMass.cpp \
           DiagonalMass.cpp \
           MatrixMass.cpp \
           UniformMass.cpp \
           MeshMatrixMass.cpp



LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(mass-local.cfg): include(mass-local.cfg) 
