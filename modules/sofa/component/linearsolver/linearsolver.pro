SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentlinearsolver

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_LINEARSOLVER

HEADERS +=  \
	   ShewchukPCGLinearSolver.h \
           CGLinearSolver.h \
           PCGLinearSolver.h \
           JacobiPreconditioner.h \
           JacobiPreconditioner.inl \
           BlockJacobiPreconditioner.h \
           BlockJacobiPreconditioner.inl \
           SSORPreconditioner.h \
           CholeskySolver.h \
           LULinearSolver.h \
           BTDLinearSolver.h \
           FullVector.h \
           FullMatrix.h \
           DiagonalMatrix.h \
           SparseMatrix.h \
           CompressedRowSparseMatrix.h \
	   GraphScatteredTypes.h \
           MatrixLinearSolver.h \
	   ParallelMatrixLinearSolver.h \
	   ParallelMatrixLinearSolver.inl \
           NewMatVector.h \
           NewMatMatrix.h \
           MatrixExpr.h \
           MultiCGLinearSolver.h \
           matrix_bloc_traits.h

SOURCES += \ 
	   ShewchukPCGLinearSolver.cpp \ 
	   initLinearSolver.cpp \
           CGLinearSolver.cpp \
           PCGLinearSolver.cpp \
           JacobiPreconditioner.cpp \
           BlockJacobiPreconditioner.cpp \
           SSORPreconditioner.cpp \
           CholeskySolver.cpp \
           LULinearSolver.cpp \
           BTDLinearSolver.cpp \
           FullVector.cpp \
	   GraphScatteredTypes.cpp \
           MatrixLinearSolver.cpp \
	   ParallelMatrixLinearSolver.cpp \
           MultiCGLinearSolver.cpp


contains(DEFINES,SOFA_HAVE_MKL){

    HEADERS += \
	MKLMatrix.h \
	MKLVector.h

}

contains(DEFINES, SOFA_HAVE_EIGEN2){        
	HEADERS +=  LagrangeMultiplierComputation.h 
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

contains(DEFINES,SOFA_HAVE_PARDISO){

    HEADERS +=  \
               SparsePARDISOSolver.h

    SOURCES +=  \
               SparsePARDISOSolver.cpp

}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(linearsolver-local.cfg) 
