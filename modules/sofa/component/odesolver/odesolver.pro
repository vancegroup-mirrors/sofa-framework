SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentodesolver

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_ODESOLVER

HEADERS += initOdeSolver.h \
           CentralDifferenceSolver.h \
           DampVelocitySolver.h \
           EulerSolver.h \
           EulerImplicitSolver.h \
           NewmarkImplicitSolver.h \
#           OdeSolverImpl.h \
           RungeKutta2Solver.h \
           RungeKutta4Solver.h \
           StaticSolver.h

SOURCES += initOdeSolver.cpp \
           CentralDifferenceSolver.cpp \
           DampVelocitySolver.cpp \
           EulerSolver.cpp \
           EulerImplicitSolver.cpp \
           NewmarkImplicitSolver.cpp \
#           OdeSolverImpl.cpp \
           RungeKutta2Solver.cpp \
           RungeKutta4Solver.cpp \
           StaticSolver.cpp


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(odesolver-local.cfg) 
