SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentforcefield

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_FORCEFIELD

HEADERS += initForceField.h \
           BeamFEMForceField.h \
           BeamFEMForceField.inl \
           BoxStiffSpringForceField.h \
           BoxStiffSpringForceField.inl \
           ConstantForceField.h \
           ConstantForceField.inl \
           DistanceGridForceField.h \
           DistanceGridForceField.inl \
           EdgePressureForceField.h \
           EdgePressureForceField.inl \
           FrameSpringForceField.h \
           FrameSpringForceField.inl \
           LennardJonesForceField.h \
           LennardJonesForceField.inl \
           SPHFluidForceField.h \
           SPHFluidForceField.inl \
           OscillatingTorsionPressureForceField.h \
           OscillatingTorsionPressureForceField.inl \
           ParticlesRepulsionForceField.h \
           ParticlesRepulsionForceField.inl \
           PlaneForceField.h \
           PlaneForceField.inl \
           SphereForceField.h \
           SphereForceField.inl \
           ConicalForceField.h \
           ConicalForceField.inl \
           EllipsoidForceField.h \
           EllipsoidForceField.inl \
           PenalityContactForceField.h \
           PenalityContactForceField.inl \
           QuadBendingSprings.h \
           QuadBendingSprings.inl \
           QuadularBendingSprings.h \
           QuadularBendingSprings.inl \
           RestShapeSpringsForceField.h \
           RestShapeSpringsForceField.inl \
           SpringForceField.h \
           SpringForceField.inl \
           StiffSpringForceField.h \
           StiffSpringForceField.inl \
           SurfacePressureForceField.h \
           SurfacePressureForceField.inl \
           JointSpringForceField.h \
           JointSpringForceField.inl \
           MeshSpringForceField.h \
           MeshSpringForceField.inl \
           RegularGridSpringForceField.h \
           RegularGridSpringForceField.inl \
           TensorForceField.h \
           TetrahedronFEMForceField.h \
           TetrahedronFEMForceField.inl \
           HexahedronFEMForceField.h \
           HexahedronFEMForceField.inl \
           HexahedralFEMForceField.h \
           HexahedralFEMForceField.inl \
           HexahedronFEMForceFieldAndMass.h \
           HexahedronFEMForceFieldAndMass.inl \
           TetrahedralTensorMassForceField.h \
           TetrahedralTensorMassForceField.inl \
           TetrahedralCorotationalFEMForceField.h \
           TetrahedralCorotationalFEMForceField.inl \
           TriangleBendingSprings.h \
           TriangleBendingSprings.inl \
           TriangularBendingSprings.h \
           TriangularBendingSprings.inl \
           TriangleFEMForceField.h \
           TriangularFEMForceField.h \
           TriangularAnisotropicFEMForceField.h \
           TrianglePressureForceField.h \
           TrianglePressureForceField.inl \
           TriangularBiquadraticSpringsForceField.h \
           TriangularBiquadraticSpringsForceField.inl \
           TriangularQuadraticSpringsForceField.h \
           TriangularQuadraticSpringsForceField.inl \
           TriangularTensorMassForceField.h \
           TriangularTensorMassForceField.inl \
           VaccumSphereForceField.h \
           VaccumSphereForceField.inl \
           VectorSpringForceField.h \
           VectorSpringForceField.inl

           
SOURCES += initForceField.cpp \
           BeamFEMForceField.cpp \
           BoxStiffSpringForceField.cpp \
           ConstantForceField.cpp \
           DistanceGridForceField.cpp \
           EdgePressureForceField.cpp \
           LennardJonesForceField.cpp \
           PlaneForceField.cpp \
           SphereForceField.cpp \
           ConicalForceField.cpp \
           EllipsoidForceField.cpp \
           FrameSpringForceField.cpp \
           SPHFluidForceField.cpp \
           OscillatingTorsionPressureForceField.cpp \	   
           ParticlesRepulsionForceField.cpp \
           SpringForceField.cpp \
           StiffSpringForceField.cpp \
           SurfacePressureForceField.cpp \
           JointSpringForceField.cpp \
           PenalityContactForceField.cpp \
           MeshSpringForceField.cpp \
           QuadularBendingSprings.cpp \
           QuadBendingSprings.cpp \
           RegularGridSpringForceField.cpp \
           RestShapeSpringsForceField.cpp \
           TetrahedronFEMForceField.cpp \
           HexahedronFEMForceField.cpp \
           HexahedralFEMForceField.cpp \
           HexahedronFEMForceFieldAndMass.cpp \
           TetrahedralTensorMassForceField.cpp \
           TetrahedralCorotationalFEMForceField.cpp \
           TriangleBendingSprings.cpp \
           TriangularBendingSprings.cpp \
           TriangleFEMForceField.cpp \
           TriangularFEMForceField.cpp \
           TriangularAnisotropicFEMForceField.cpp \
           TrianglePressureForceField.cpp \
           TriangularBiquadraticSpringsForceField.cpp \
           TriangularQuadraticSpringsForceField.cpp \
           TriangularTensorMassForceField.cpp \
           VaccumSphereForceField.cpp \
           VectorSpringForceField.cpp
           

contains(DEFINES,SOFA_TEST_FRICTION){

    HEADERS += PenalityContactFrictionForceField.h \
           PenalityContactFrictionForceField.inl

    SOURCES += PenalityContactFrictionForceField.cpp 

}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(forcefield-local.cfg) 
