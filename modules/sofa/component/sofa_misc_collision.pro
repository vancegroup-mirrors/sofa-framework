load(sofa/pre)

TEMPLATE = lib
TARGET = sofa_misc_collision

DEFINES += SOFA_BUILD_MISC_COLLISION

HEADERS += collision/TriangleModelInRegularGrid.h \
           collision/RigidContactMapper.h \
           collision/RigidContactMapper.inl \
           collision/TreeCollisionGroupManager.h \
           collision/RuleBasedContactManager.h \
           collision/DefaultCollisionGroupManager.h \
           collision/SolverMerger.h

SOURCES += collision/TriangleModelInRegularGrid.cpp \
           collision/RigidContactMapper.cpp \
           collision/TreeCollisionGroupManager.cpp \
           collision/RuleBasedContactManager.cpp \
           collision/DefaultCollisionGroupManager.cpp \
           collision/SolverMerger.cpp


contains(DEFINES,SOFA_DEV){

HEADERS += collision/ContinuousIntersection.h \

SOURCES += collision/ContinuousIntersection.cpp \

}

contains(DEFINES,SOFA_SMP){
HEADERS += collision/ParallelCollisionPipeline.h

SOURCES += collision/ParallelCollisionPipeline.cpp
}

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_INSTALL_INC_DIR/applications

#exists(component-local.cfg): include(component-local.cfg)

load(sofa/post)
 
