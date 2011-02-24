SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentengine

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_ENGINE

HEADERS += initEngine.h \
           AverageCoord.h \
           AverageCoord.inl \
           BoxROI.h \
           BoxROI.inl \
					Distances.h \
					Distances.inl \
           PlaneROI.h \
           PlaneROI.inl \
	   SphereROI.h \
	   SphereROI.inl \
           ExtrudeSurface.h \
           ExtrudeSurface.inl \
           GenerateRigidMass.h \
           GenerateRigidMass.inl \
           GroupFilterYoungModulus.h \
           GroupFilterYoungModulus.inl \
           MergePoints.h \
           MergePoints.inl \
           MergeSets.h \
           MergeSets.inl \
	   MeshBarycentricMapperEngine.h \
	   MeshBarycentricMapperEngine.inl \
	   TransformPosition.h \
	   TransformPosition.inl \
	   TransformEngine.h \
	   TransformEngine.inl \
           PointsFromIndices.h \
           PointsFromIndices.inl \
           ValuesFromIndices.h \
           ValuesFromIndices.inl \
           IndicesFromValues.h \
           IndicesFromValues.inl \
           IndexValueMapper.h \
           IndexValueMapper.inl \
           JoinPoints.h \
           JoinPoints.inl \
           MapIndices.h \
           MapIndices.inl \
           RandomPointDistributionInSurface.h \
           RandomPointDistributionInSurface.inl \
           Spiral.h \
           Spiral.inl \
           Vertex2Frame.h \
           Vertex2Frame.inl \
           TextureInterpolation.h \
           TextureInterpolation.inl \
           SubsetTopology.h \
           SubsetTopology.inl \
           RigidEngine.h \
           RigidEngine.inl \
           ValuesFromPositions.h \
           ValuesFromPositions.inl

SOURCES += initEngine.cpp \
           AverageCoord.cpp \
           BoxROI.cpp \
					Distances.cpp \
           PlaneROI.cpp \
	   SphereROI.cpp \
           ExtrudeSurface.cpp \
           GenerateRigidMass.cpp \
           GroupFilterYoungModulus.cpp \
           MergePoints.cpp \
           MergeSets.cpp \
	   MeshBarycentricMapperEngine.cpp \
	   TransformPosition.cpp \
           TransformEngine.cpp \
           PointsFromIndices.cpp \
           ValuesFromIndices.cpp \
           IndicesFromValues.cpp \
           IndexValueMapper.cpp \
           JoinPoints.cpp \
           MapIndices.cpp \
           RandomPointDistributionInSurface.cpp \
           Spiral.cpp \
           Vertex2Frame.cpp \
           TextureInterpolation.cpp \
           SubsetTopology.cpp \
           RigidEngine.cpp \
           ValuesFromPositions.cpp
           
      

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentcollision$$LIBSUFFIX
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(engine-local.cfg): include(engine-local.cfg) 
