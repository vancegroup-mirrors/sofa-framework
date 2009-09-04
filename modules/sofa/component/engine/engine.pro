SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentengine$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_ENGINE

HEADERS += initEngine.h \
           BoxROI.h \
           BoxROI.inl \
           ExtrudeSurface.h \
           ExtrudeSurface.inl \
           MergePoints.h \
           MergePoints.inl \
	   TransformPosition.h \
	   TransformPosition.inl \
           PlaneROI.h \
           PlaneROI.inl \
           PointsFromIndices.h \
           PointsFromIndices.inl \
           RandomPointDistributionInSurface.h \
           RandomPointDistributionInSurface.inl \
           Spiral.h \
           Spiral.inl \
           TrianglesInBoxROI.h \
           TrianglesInBoxROI.inl \
           TrianglesInSphereROI.h \
           TrianglesInSphereROI.inl \
           TrianglesInPlaneROI.h \
           TrianglesInPlaneROI.inl \
           Vertex2Frame.h \
           Vertex2Frame.inl

SOURCES += initEngine.cpp \
           BoxROI.cpp \
           ExtrudeSurface.cpp \
           MergePoints.cpp \
           PlaneROI.cpp \
	   TransformPosition.cpp \
           PointsFromIndices.cpp \
           RandomPointDistributionInSurface.cpp \
           Spiral.cpp \
           TrianglesInBoxROI.cpp \
           TrianglesInSphereROI.cpp \
           TrianglesInPlaneROI.cpp \
           Vertex2Frame.cpp

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentcollision$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(engine-local.cfg) 
