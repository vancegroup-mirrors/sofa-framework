SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponentcollision$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

CONFIG -= staticlib
CONFIG += dll

DEFINES += SOFA_BUILD_COMPONENT_COLLISION

HEADERS += initCollision.h \
           BarycentricContactMapper.h \
           BarycentricContactMapper.inl \
           BarycentricPenalityContact.h \
           BarycentricPenalityContact.inl \
           BruteForceDetection.h \
           CarvingManager.h \
           ContinuousIntersection.h \
           ContinuousTriangleIntersection.h \
           Cube.h \
           CubeModel.h \
           TopologicalChangeManager.h \
           DefaultCollisionGroupManager.h \
           DefaultContactManager.h \
           DefaultPipeline.h \
           DiscreteIntersection.h \
           DiscreteIntersection.inl \
           DistanceGridCollisionModel.h \
           FrictionContact.h \
           FrictionContact.inl \
           Line.h \
           LineModel.h \
           LocalMinDistance.h \
           MinProximityIntersection.h \
           NewProximityIntersection.h \
           NewProximityIntersection.inl \
           Point.h \
           PointModel.h \
           Ray.h \
           RayContact.h \
           RayModel.h \
           RayPickInteractor.h \
           RayPickInteractor.inl \
           RayTriangleIntersection.h \
           RuleBasedContactManager.h \
           SpatialGridPointModel.h \
           Sphere.h \
           SphereModel.h \
           SphereTreeModel.h \
           TetrahedronModel.h \
           Triangle.h \
           TriangleModel.h \
           RayTraceDetection.h \
           TriangleOctree.h \
           TriangleOctreeModel.h

SOURCES += initCollision.cpp \
           BarycentricContactMapper.cpp \
           BarycentricPenalityContact.cpp \
           BruteForceDetection.cpp \
           ContinuousIntersection.cpp \
           ContinuousTriangleIntersection.cpp \
           CubeModel.cpp \
           TopologicalChangeManager.cpp \
           CarvingManager.cpp \
           DefaultCollisionGroupManager.cpp \
           DefaultContactManager.cpp \
           DefaultPipeline.cpp \
           DiscreteIntersection.cpp \
           DistanceGridCollisionModel.cpp \
           LineModel.cpp \
           FrictionContact.cpp \
           LocalMinDistance.cpp \
           MinProximityIntersection.cpp \
           NewProximityIntersection.cpp \
           PointModel.cpp \
           RayContact.cpp \
           RayModel.cpp \
           RayPickInteractor.cpp \
           RayTriangleIntersection.cpp \
           RuleBasedContactManager.cpp \
           SpatialGridPointModel.cpp \
           SphereModel.cpp \
           SphereTreeModel.cpp \
           TetrahedronModel.cpp \
           TriangleModel.cpp \
           RayTraceDetection.cpp \
           TriangleOctree.cpp \
           TriangleOctreeModel.cpp 
           


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentmapping$$LIBSUFFIX
LIBS += -lsofacomponentconstraint$$LIBSUFFIX
LIBS += -lsofacomponentvisualmodel$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(collision-local.cfg) 
