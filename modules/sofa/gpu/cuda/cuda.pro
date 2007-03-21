# Target is a library: sofagpucuda

SOFA_DIR = ../../../..
TEMPLATE = lib
include($$SOFA_DIR/sofa.cfg)

TARGET = sofagpucuda$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

HEADERS += mycuda.h \
           CudaTypes.h \
           CudaCommon.h \
           CudaMath.h \
           CudaMechanicalObject.h \
           CudaMechanicalObject.inl \
           CudaUniformMass.h \
           CudaUniformMass.inl \
           CudaFixedConstraint.h \
           CudaFixedConstraint.inl \
           CudaSpringForceField.h \
           CudaSpringForceField.inl \
           CudaPlaneForceField.h \
           CudaPlaneForceField.inl

SOURCES += mycuda.cpp \
           CudaMechanicalObject.cpp \
           CudaUniformMass.cpp \
           CudaFixedConstraint.cpp \
           CudaSpringForceField.cpp \
           CudaPlaneForceField.cpp

CUDA_SOURCES += mycuda.cu \
           CudaMechanicalObject.cu \
           CudaUniformMass.cu \
           CudaFixedConstraint.cu \
           CudaSpringForceField.cu \
           CudaPlaneForceField.cu

########################################################################
#  CUDA
########################################################################
win32 {
  INCLUDEPATH += $(CUDA_INC_DIR)
  QMAKE_LIBDIR += $(CUDA_LIB_DIR)
  LIBS += -lcudart

  cuda.output = $$OBJECTS_DIR/${QMAKE_FILE_BASE}_cuda.obj
  cuda.commands = $(CUDA_BIN_DIR)/nvcc.exe -c -Xcompiler $$join(QMAKE_CXXFLAGS,",") $$join(INCLUDEPATH,'" -I "','-I "','"') ${QMAKE_FILE_NAME} -o ${QMAKE_FILE_OUT}
}
unix {
  # auto-detect CUDA path
  CUDA_DIR = $$system(which nvcc | sed 's,/bin/nvcc$,,')
  INCLUDEPATH += $$CUDA_DIR/include
  QMAKE_LIBDIR += $$CUDA_DIR/lib
  LIBS += -lcudart

  cuda.output = ${OBJECTS_DIR}${QMAKE_FILE_BASE}_cuda.obj
  cuda.commands = nvcc -c -Xcompiler $$join(QMAKE_CXXFLAGS,",") $$join(INCLUDEPATH,'" -I "','-I "','"') ${QMAKE_FILE_NAME} -o ${QMAKE_FILE_OUT}
  cuda.depends = nvcc -M -Xcompiler $$join(QMAKE_CXXFLAGS,",") $$join(INCLUDEPATH,'" -I "','-I "','"') ${QMAKE_FILE_NAME} | tail +2 | sed "s,^ *,,; s/ \\\\$//" | paste -s -d " " | sed "s,/usr/[^ ]*,,g" | tee dep-${QMAKE_FILE_NAME}
# | tail +2 | sed "s,^    ,," | tee dep-${QMAKE_FILE_NAME}
# | tail +2 | sed "s,^    ,  ,"
# | sed "s,^.*: ,," | sed "s,^ *,," | tr -d '\\\n' | tee dep-${QMAKE_FILE_NAME}
}
cuda.input = CUDA_SOURCES
QMAKE_EXTRA_UNIX_COMPILERS += cuda

########################################################################
