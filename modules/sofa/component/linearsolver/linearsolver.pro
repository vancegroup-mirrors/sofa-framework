SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentlinearsolver

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_LINEARSOLVER

HEADERS +=  \
           CGLinearSolver.h \
           PCGLinearSolver.h \
           JacobiPreconditioner.h \
           BlockJacobiPreconditioner.h \
           SSORPreconditioner.h \
           CholeskySolver.h \
           LULinearSolver.h \
           BTDLinearSolver.h \
           FullVector.h \
           FullMatrix.h \
           SparseMatrix.h \
           CompressedRowSparseMatrix.h \
           MatrixLinearSolver.h \
           NewMatVector.h \
           NewMatMatrix.h 

SOURCES += initLinearSolver.cpp \
           CGLinearSolver.cpp \
           PCGLinearSolver.cpp \
           JacobiPreconditioner.cpp \
           BlockJacobiPreconditioner.cpp \
           SSORPreconditioner.cpp \
           CholeskySolver.cpp \
           LULinearSolver.cpp \
           BTDLinearSolver.cpp \
           FullVector.cpp \
           MatrixLinearSolver.cpp 


contains(DEFINES,SOFA_HAVE_MKL){

    HEADERS += \
	MKLMatrix.h \
	MKLVector.h

}



contains(DEFINES,SOFA_HAVE_CSPARSE){

    HEADERS +=  \
               SparseCholeskySolver.h \
               SparseLUSolver.h \
               SparseLDLSolver.h

    SOURCES +=  \
               SparseCholeskySolver.cpp \
               SparseLUSolver.cpp \
               SparseLDLSolver.cpp
               
}

contains(DEFINES,SOFA_HAVE_TAUCS){

    HEADERS +=  \
               SparseTAUCSSolver.h

    SOURCES +=  \
               SparseTAUCSSolver.cpp

}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(linearsolver-local.cfg) 
