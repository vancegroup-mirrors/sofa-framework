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
	   CompressedRowSparseMatrix.inl \
	   GraphScatteredTypes.h \
           DefaultMultiMatrixAccessor.h \
           MatrixLinearSolver.h \
	   ParallelMatrixLinearSolver.h \
	   ParallelMatrixLinearSolver.inl \
           NewMatVector.h \
           NewMatMatrix.h \
           MatrixExpr.h \
	   GenerateBenchSolver.h \
	   GenerateBenchSolver.inl \
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
           DefaultMultiMatrixAccessor.cpp \
           MatrixLinearSolver.cpp \
	   GenerateBenchSolver.cpp


contains(DEFINES,SOFA_HAVE_MKL){

    HEADERS += \
	MKLMatrix.h \
	MKLVector.h

}



contains(DEFINES, SOFA_HAVE_EIGEN2){
	HEADERS += EigenMatrixManipulator.h

	SOURCES += EigenMatrixManipulator.cpp
} 

contains(DEFINES, SOFA_SMP){ # BEGIN SOFA_SMP
	HEADERS += ParallelCGLinearSolver.h \
						 ParallelCGLinearSolver.inl

	SOURCES += ParallelCGLinearSolver.cpp

} # END SOFA_SMP

contains(DEFINES,SOFA_HAVE_CSPARSE){

    HEADERS +=  \
               SparseCholeskySolver.h \
               SparseLUSolver.h \
               SparseLDLSolver.h \
	       SparseLDLSolver.inl

    SOURCES +=  \
               SparseCholeskySolver.cpp \
               SparseLUSolver.cpp \
               SparseLDLSolver.cpp
               
}

contains(DEFINES,SOFA_HAVE_TAUCS){
win32{
  # BLAS
  LIBS *= -lblas_win32_MT
  # LAPACK
  LIBS *= -llapack_win32_MT
  }
    HEADERS +=  \
               SparseTAUCSSolver.h \
	       SparseTAUCSLUSolver.h \
	       IncompleteTAUCSSolver.h

    SOURCES +=  \
               SparseTAUCSSolver.cpp \
	       SparseTAUCSLUSolver.cpp \
	       IncompleteTAUCSSolver.cpp

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
LIBS += -lsofacomponentbase$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(linearsolver-local.cfg): include(linearsolver-local.cfg) 
