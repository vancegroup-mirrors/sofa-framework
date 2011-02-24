SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentcollision

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_COLLISION

HEADERS += initCollision.h \
#           AddFramePerformer.h \
#           AddFramePerformer.inl \
           AttachBodyPerformer.h \
           AttachBodyPerformer.inl \
           BaseContactMapper.h \
           BarycentricContactMapper.h \
           BarycentricContactMapper.inl \
           RigidContactMapper.h \
           RigidContactMapper.inl \
           BarycentricPenalityContact.h \
           BarycentricPenalityContact.inl \
           BruteForceDetection.h \
           CarvingManager.h \
           ComponentMouseInteraction.h \
           ComponentMouseInteraction.inl \
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
           FixParticlePerformer.h \
           FixParticlePerformer.inl \
           FrictionContact.h \
           FrictionContact.inl \
           InciseAlongPathPerformer.h \
           IdentityContactMapper.h \
           IdentityContactMapper.inl \
           InteractionPerformer.h \
           Line.h \
           LineLocalMinDistanceFilter.h \
           LineModel.h \
           LMDNewProximityIntersection.h \
           LMDNewProximityIntersection.inl \
           LocalMinDistance.h \
	   LocalMinDistanceFilter.h \
           MinProximityIntersection.h \
           MouseInteractor.h \
           MouseInteractor.inl \
           NewProximityIntersection.h \
           NewProximityIntersection.inl \
           Point.h \
	   PointLocalMinDistanceFilter.h \
           PointModel.h \
           SuturePointPerformer.h \
           SuturePointPerformer.inl \
           Ray.h \
           RayContact.h \
           RayModel.h \
#           RayPickInteractor.h \
#           RayPickInteractor.inl \
           RayTriangleIntersection.h \
           RemovePrimitivePerformer.h \
           RemovePrimitivePerformer.inl \
           RuleBasedContactManager.h \
           SolverMerger.h \
           SpatialGridPointModel.h \
           Sphere.h \
           SphereModel.h \
		   SphereModel.inl \
           SphereTreeModel.h \
           SubsetContactMapper.h \
           SubsetContactMapper.inl \
           TetrahedronModel.h \
           TreeCollisionGroupManager.h \
           Triangle.h \
	   TriangleLocalMinDistanceFilter.h \
           TriangleModel.h \
           TriangleModelInRegularGrid.h \
           RayTraceDetection.h \
           TriangleOctree.h \
           TriangleOctreeModel.h \

SOURCES += initCollision.cpp \
#           AddFramePerformer.cpp \
           AttachBodyPerformer.cpp \
           BaseContactMapper.cpp \
           BarycentricContactMapper.cpp \
           RigidContactMapper.cpp \
           BarycentricPenalityContact.cpp \
           BruteForceDetection.cpp \     
           ComponentMouseInteraction.cpp \
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
           IdentityContactMapper.cpp \
           InciseAlongPathPerformer.cpp \
           LineModel.cpp \
           LMDNewProximityIntersection.cpp \
           FixParticlePerformer.cpp \
           FrictionContact.cpp \
	   InteractionPerformer.cpp \
           LineLocalMinDistanceFilter.cpp \
           LocalMinDistance.cpp \
           LocalMinDistanceFilter.cpp \
           MinProximityIntersection.cpp \
           MouseInteractor.cpp \
           NewProximityIntersection.cpp \
           PointLocalMinDistanceFilter.cpp \
           PointModel.cpp \
           SuturePointPerformer.cpp \
           RayContact.cpp \
           RayModel.cpp \
#           RayPickInteractor.cpp \  
           RayTriangleIntersection.cpp \
           RemovePrimitivePerformer.cpp \
           RuleBasedContactManager.cpp \
           SolverMerger.cpp \
           SpatialGridPointModel.cpp \
           SphereModel.cpp \
           SphereTreeModel.cpp \
           SubsetContactMapper.cpp \
           TetrahedronModel.cpp \
           TreeCollisionGroupManager.cpp \
	   TriangleLocalMinDistanceFilter.cpp \
           TriangleModel.cpp \
           TriangleModelInRegularGrid.cpp \
           RayTraceDetection.cpp \
           TriangleOctree.cpp \
           TriangleOctreeModel.cpp \



contains(DEFINES,SOFA_HAVE_EIGEN2){

HEADERS += BarycentricDistanceLMConstraintContact.h \
           BarycentricDistanceLMConstraintContact.inl 
SOURCES += BarycentricDistanceLMConstraintContact.cpp

}
contains(DEFINES,SOFA_SMP){

HEADERS += \
           ParallelCollisionPipeline.h
SOURCES += \
           ParallelCollisionPipeline.cpp

}


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentinteractionforcefield$$LIBSUFFIX
LIBS += -lsofacomponentmapping$$LIBSUFFIX
LIBS += -lsofacomponentprojectiveconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentconstraintset$$LIBSUFFIX
LIBS += -lsofacomponentvisualmodel$$LIBSUFFIX


LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(collision-local.cfg): include(collision-local.cfg) 
