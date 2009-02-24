SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)
TARGET = sofacomponent$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

HEADERS += init.h \
	   MeshLoader.h \
           MappedObject.h \
           MappedObject.inl \
           MechanicalObject.h \
           MechanicalObject.inl \
           behaviormodel/eulerianfluid/Fluid2D.h \
           behaviormodel/eulerianfluid/Fluid3D.h \
           behaviormodel/eulerianfluid/Grid2D.h \
           behaviormodel/eulerianfluid/Grid3D.h \
           collision/BarycentricContactMapper.h \
           collision/BarycentricContactMapper.inl \
           collision/BarycentricPenalityContact.h \
           collision/BarycentricPenalityContact.inl \
           collision/BruteForceDetection.h \
           collision/CarvingManager.h \
           collision/ContinuousIntersection.h \
           collision/ContinuousTriangleIntersection.h \
           collision/Cube.h \
           collision/CubeModel.h \
           collision/TopologicalChangeManager.h \
           collision/DefaultCollisionGroupManager.h \
           collision/DefaultContactManager.h \
           collision/DefaultPipeline.h \
           collision/DiscreteIntersection.h \
           collision/DiscreteIntersection.inl \
           collision/DistanceGridCollisionModel.h \
           collision/FrictionContact.h \
           collision/FrictionContact.inl \
           collision/Line.h \
           collision/LineModel.h \
           collision/LocalMinDistance.h \
           collision/MinProximityIntersection.h \
           collision/NewProximityIntersection.h \
           collision/NewProximityIntersection.inl \
           collision/Point.h \
           collision/PointModel.h \
           collision/proximity.h \
           collision/Ray.h \
           collision/RayContact.h \
           collision/RayModel.h \
           collision/RayPickInteractor.h \
           collision/RayPickInteractor.inl \
           collision/RayTriangleIntersection.h \
           collision/SpatialGridPointModel.h \
           collision/Sphere.h \
           collision/SphereModel.h \
           collision/SphereModel.inl \
           collision/SphereTreeModel.h \
           collision/TetrahedronModel.h \
           collision/Triangle.h \
           collision/TriangleModel.h \
           collision/RayTraceDetection.h \
           collision/TriangleOctree.h \
           collision/TriangleOctreeModel.h \
           constraint/AttachConstraint.h \
           constraint/AttachConstraint.inl \
           constraint/BoxConstraint.h \
           constraint/BoxConstraint.inl \
           constraint/FixedConstraint.h \
           constraint/FixedConstraint.inl \
           constraint/FixedPlaneConstraint.h \
           constraint/FixedPlaneConstraint.inl \
           constraint/OscillatorConstraint.h \
           constraint/OscillatorConstraint.inl \
           constraint/LinearSolverConstraintCorrection.h \
           constraint/LinearSolverConstraintCorrection.inl \
           constraint/ParabolicConstraint.h \
           constraint/ParabolicConstraint.inl \
           constraint/PrecomputedConstraintCorrection.h \
           constraint/PrecomputedConstraintCorrection.inl \
           constraint/UncoupledConstraintCorrection.h \
           constraint/UncoupledConstraintCorrection.inl \
           constraint/UnilateralInteractionConstraint.h \
           constraint/UnilateralInteractionConstraint.inl \
           constraint/LinearMovementConstraint.h \
           constraint/LinearMovementConstraint.inl \
           container/ArticulatedHierarchyContainer.h \
           container/ArticulatedHierarchyContainer.inl \
           container/SpatialGridContainer.h \
           container/SpatialGridContainer.inl \
           contextobject/CoordinateSystem.h \
           contextobject/Gravity.h \
           controller/ArticulatedHierarchyController.h \
           controller/ArticulatedHierarchyBVHController.h \
           controller/Controller.h \
           controller/EdgeSetController.h \
           controller/EdgeSetController.inl \
           controller/MechanicalStateController.h \
           controller/MechanicalStateController.inl \
           forcefield/BeamFEMForceField.h \
           forcefield/BeamFEMForceField.inl \
           forcefield/BoxStiffSpringForceField.h \
           forcefield/BoxStiffSpringForceField.inl \
           forcefield/ConstantForceField.h \
           forcefield/ConstantForceField.inl \
           forcefield/BoxConstantForceField.h \
           forcefield/BoxConstantForceField.inl \
           forcefield/EdgePressureForceField.h \
           forcefield/EdgePressureForceField.inl \
           forcefield/FrameSpringForceField.h \
           forcefield/FrameSpringForceField.inl \
           forcefield/LennardJonesForceField.h \
           forcefield/LennardJonesForceField.inl \
           forcefield/SPHFluidForceField.h \
           forcefield/SPHFluidForceField.inl \
           forcefield/PlaneForceField.h \
           forcefield/PlaneForceField.inl \
           forcefield/SphereForceField.h \
           forcefield/SphereForceField.inl \
           forcefield/ConicalForceField.h \
           forcefield/ConicalForceField.inl \
           forcefield/EllipsoidForceField.h \
           forcefield/EllipsoidForceField.inl \
           forcefield/PenalityContactForceField.h \
           forcefield/PenalityContactForceField.inl \
           forcefield/QuadBendingSprings.h \
           forcefield/QuadBendingSprings.inl \
           forcefield/QuadularBendingSprings.h \
           forcefield/QuadularBendingSprings.inl \
           forcefield/SpringForceField.h \
           forcefield/SpringForceField.inl \
           forcefield/StiffSpringForceField.h \
           forcefield/StiffSpringForceField.inl \
           forcefield/JointSpringForceField.h \
           forcefield/JointSpringForceField.inl \
           forcefield/MeshSpringForceField.h \
           forcefield/MeshSpringForceField.inl \
           forcefield/RegularGridSpringForceField.h \
           forcefield/RegularGridSpringForceField.inl \
           forcefield/TensorForceField.h \
           forcefield/TetrahedronFEMForceField.h \
           forcefield/TetrahedronFEMForceField.inl \
           forcefield/HexahedronFEMForceField.h \
           forcefield/HexahedronFEMForceField.inl \
           forcefield/HexahedralFEMForceField.h \
           forcefield/HexahedralFEMForceField.inl \
           forcefield/TetrahedralTensorMassForceField.h \
           forcefield/TetrahedralTensorMassForceField.inl \
           forcefield/TetrahedralCorotationalFEMForceField.h \
           forcefield/TetrahedralCorotationalFEMForceField.inl \
           forcefield/TriangleBendingSprings.h \
           forcefield/TriangleBendingSprings.inl \
           forcefield/TriangularBendingSprings.h \
           forcefield/TriangularBendingSprings.inl \
           forcefield/TriangleFEMForceField.h \
           forcefield/TriangularFEMForceField.h \
           forcefield/TriangularAnisotropicFEMForceField.h \
           forcefield/TrianglePressureForceField.h \
           forcefield/TrianglePressureForceField.inl \
           forcefield/TriangularBiquadraticSpringsForceField.h \
           forcefield/TriangularBiquadraticSpringsForceField.inl \
           forcefield/TriangularQuadraticSpringsForceField.h \
           forcefield/TriangularQuadraticSpringsForceField.inl \
           forcefield/TriangularTensorMassForceField.h \
           forcefield/TriangularTensorMassForceField.inl \
           forcefield/VectorSpringForceField.h \
           forcefield/VectorSpringForceField.inl \
           interactionforcefield/RepulsiveSpringForceField.h \
           interactionforcefield/RepulsiveSpringForceField.inl \
           interactionforcefield/InteractionEllipsoidForceField.h \
           interactionforcefield/InteractionEllipsoidForceField.inl \
           linearsolver/CGLinearSolver.h \
           linearsolver/LULinearSolver.h \
           linearsolver/BTDLinearSolver.h \
           linearsolver/FullVector.h \
           linearsolver/SparseMatrix.h \
           linearsolver/NewMatVector.h \
           linearsolver/NewMatMatrix.h \
           mapping/ArticulatedSystemMapping.h \
           mapping/ArticulatedSystemMapping.inl \
           mapping/BarycentricMapping.h \
           mapping/BarycentricMapping.inl \
           mapping/BeamLinearMapping.h \
           mapping/BeamLinearMapping.inl \
           mapping/CenterOfMassMapping.h \
           mapping/CenterOfMassMapping.inl \
           mapping/CurveMapping.h \
           mapping/CurveMapping.inl \
           mapping/IdentityMapping.h \
           mapping/IdentityMapping.inl \
           mapping/ImplicitSurfaceMapping.h \
           mapping/ImplicitSurfaceMapping.inl \
           mapping/LaparoscopicRigidMapping.h \
           mapping/LaparoscopicRigidMapping.inl \
           mapping/LineSetSkinningMapping.h \
           mapping/LineSetSkinningMapping.inl \
           mapping/RigidMapping.h \
           mapping/RigidMapping.inl \
           mapping/RigidRigidMapping.h \
           mapping/RigidRigidMapping.inl \
           mapping/SkinningMapping.h \
           mapping/SkinningMapping.inl \
           mapping/SPHFluidSurfaceMapping.h \
           mapping/SPHFluidSurfaceMapping.inl \
           mapping/SubsetMapping.h \
           mapping/SubsetMapping.inl \
           mapping/TubularMapping.h \
           mapping/TubularMapping.inl \
           mapping/VoidMapping.h \
           mass/AddMToMatrixFunctor.h \
           mass/DiagonalMass.h \
           mass/DiagonalMass.inl \
           mass/MatrixMass.h \
           mass/MatrixMass.inl \
           mass/UniformMass.h \
           mass/UniformMass.inl \
           misc/CompareState.h \
           misc/Monitor.h \
           misc/Monitor.inl \
           misc/ParticleSink.h \
           misc/ParticleSource.h \
           misc/ReadState.h \
           misc/ReadState.inl \
           misc/WriteState.h \
           misc/WriteState.inl \
           mastersolver/DefaultMasterSolver.h \
           mastersolver/MultiStepMasterSolver.h \
           mastersolver/MasterContactSolver.h \
           odesolver/CentralDifferenceSolver.h \
           odesolver/CGImplicitSolver.h \
           odesolver/DampVelocitySolver.h \
           odesolver/EulerSolver.h \
           odesolver/EulerImplicitSolver.h \
           odesolver/RungeKutta2Solver.h \
           odesolver/RungeKutta4Solver.h \
           odesolver/StaticSolver.h \
           topology/CubeTopology.h \
           topology/CylinderGridTopology.h \
           topology/Edge2QuadTopologicalMapping.h \
           topology/EdgeData.h \
           topology/EdgeData.inl \
           topology/EdgeSetGeometryAlgorithms.h \
           topology/EdgeSetTopologyAlgorithms.h \
           topology/EdgeSetTopologyChange.h \
           topology/EdgeSetTopologyContainer.h \
           topology/EdgeSetTopologyModifier.h \
           topology/EdgeSetGeometryAlgorithms.inl \
           topology/EdgeSetTopologyAlgorithms.inl \
           topology/EdgeSubsetData.h \
           topology/EdgeSubsetData.inl \
           topology/GridTopology.h \
           topology/Hexa2QuadTopologicalMapping.h \
     	   topology/HexahedronData.h \
           topology/HexahedronData.inl \
           topology/HexahedronSetGeometryAlgorithms.h \
           topology/HexahedronSetTopologyAlgorithms.h \
           topology/HexahedronSetTopologyChange.h \
           topology/HexahedronSetTopologyContainer.h \
           topology/HexahedronSetTopologyModifier.h \
           topology/HexahedronSetGeometryAlgorithms.inl \
           topology/HexahedronSetTopologyAlgorithms.inl \
           topology/ManifoldEdgeSetGeometryAlgorithms.h \
           topology/ManifoldEdgeSetTopologyAlgorithms.h \
           topology/ManifoldEdgeSetTopologyContainer.h \
           topology/ManifoldEdgeSetTopologyModifier.h \
           topology/ManifoldEdgeSetGeometryAlgorithms.inl \
           topology/ManifoldEdgeSetTopologyAlgorithms.inl \
           topology/MeshTopology.h \
           topology/PointData.h \
           topology/PointData.inl \
           topology/PointSetGeometryAlgorithms.h \
           topology/PointSetTopologyAlgorithms.h \
           topology/PointSetTopologyChange.h \
           topology/PointSetTopologyContainer.h \
           topology/PointSetTopologyModifier.h \
           topology/PointSetGeometryAlgorithms.inl \
           topology/PointSetTopologyAlgorithms.inl \
           topology/PointSubset.h \
           topology/Quad2TriangleTopologicalMapping.h \
           topology/QuadData.h \
           topology/QuadData.inl \
           topology/QuadSetGeometryAlgorithms.h \
           topology/QuadSetTopologyAlgorithms.h \
           topology/QuadSetTopologyChange.h \
           topology/QuadSetTopologyContainer.h \
           topology/QuadSetTopologyModifier.h \
           topology/QuadSetGeometryAlgorithms.inl \
           topology/QuadSetTopologyAlgorithms.inl \
           topology/RegularGridTopology.h \
           topology/SparseGridTopology.h \
           topology/Tetra2TriangleTopologicalMapping.h \
           topology/TetrahedronData.h \
           topology/TetrahedronData.inl \
           topology/TetrahedronSetGeometryAlgorithms.h \
           topology/TetrahedronSetTopologyAlgorithms.h \
           topology/TetrahedronSetTopologyChange.h \
           topology/TetrahedronSetTopologyContainer.h \
           topology/TetrahedronSetTopologyModifier.h \
           topology/TetrahedronSetGeometryAlgorithms.inl \
           topology/TetrahedronSetTopologyAlgorithms.inl \
           topology/TopologyChangedEvent.h \
           topology/Triangle2EdgeTopologicalMapping.h \
           topology/TriangleData.h \
           topology/TriangleData.inl \
           topology/TriangleSetGeometryAlgorithms.h \
           topology/TriangleSetTopologyAlgorithms.h \
           topology/TriangleSetTopologyChange.h \
           topology/TriangleSetTopologyContainer.h \
           topology/TriangleSetTopologyModifier.h \
           topology/TriangleSetGeometryAlgorithms.inl \
           topology/TriangleSetTopologyAlgorithms.inl \
           topology/TriangleSubsetData.h \
           topology/TriangleSubsetData.inl \
           visualmodel/ClipPlane.h \
           visualmodel/DrawV.h \
           visualmodel/OglModel.h \
           visualmodel/VisualModelImpl.h \
           visualmodel/Light.h \
           visualmodel/LightManager.h

SOURCES += init.cpp \
	   MeshLoader.cpp \
           MappedObject.cpp \
           MechanicalObject.cpp \
           behaviormodel/eulerianfluid/Fluid2D.cpp \
           behaviormodel/eulerianfluid/Fluid3D.cpp \
           behaviormodel/eulerianfluid/Grid2D.cpp \
           behaviormodel/eulerianfluid/Grid3D.cpp \
           collision/BarycentricContactMapper.cpp \
           collision/BarycentricPenalityContact.cpp \
           collision/BruteForceDetection.cpp \
           collision/ContinuousIntersection.cpp \
           collision/ContinuousTriangleIntersection.cpp \
           collision/CubeModel.cpp \
           collision/TopologicalChangeManager.cpp \
           collision/CarvingManager.cpp \
           collision/DefaultCollisionGroupManager.cpp \
           collision/DefaultContactManager.cpp \
           collision/DefaultPipeline.cpp \
           collision/DiscreteIntersection.cpp \
           collision/DistanceGridCollisionModel.cpp \
           collision/LineModel.cpp \
           collision/FrictionContact.cpp \
           collision/LocalMinDistance.cpp \
           collision/MinProximityIntersection.cpp \
           collision/NewProximityIntersection.cpp \
           collision/PointModel.cpp \
           collision/proximity.cpp \
           collision/RayContact.cpp \
           collision/RayModel.cpp \
           collision/RayPickInteractor.cpp \
           collision/RayTriangleIntersection.cpp \
           collision/SpatialGridPointModel.cpp \
           collision/SphereModel.cpp \
           collision/SphereTreeModel.cpp \
           collision/TetrahedronModel.cpp \
           collision/TriangleModel.cpp \
           collision/RayTraceDetection.cpp \
           collision/TriangleOctree.cpp \
           collision/TriangleOctreeModel.cpp \
           constraint/AttachConstraint.cpp \
           constraint/BoxConstraint.cpp \
           constraint/FixedConstraint.cpp \
           constraint/FixedPlaneConstraint.cpp \
           constraint/OscillatorConstraint.cpp \
           constraint/LinearSolverConstraintCorrection.cpp \
           constraint/ParabolicConstraint.cpp \
           constraint/PrecomputedConstraintCorrection.cpp \
           constraint/UncoupledConstraintCorrection.cpp \
           constraint/UnilateralInteractionConstraint.cpp \
           constraint/LinearMovementConstraint.cpp \
           container/ArticulatedHierarchyContainer.cpp \
           container/SpatialGridContainer.cpp \
           contextobject/CoordinateSystem.cpp \
           contextobject/Gravity.cpp \
           controller/ArticulatedHierarchyController.cpp \
           controller/ArticulatedHierarchyBVHController.cpp \
           controller/Controller.cpp \
           controller/EdgeSetController.cpp \
           controller/MechanicalStateController.cpp \
           forcefield/BeamFEMForceField.cpp \
           forcefield/BoxStiffSpringForceField.cpp \
           forcefield/ConstantForceField.cpp \
           forcefield/BoxConstantForceField.cpp \
           forcefield/EdgePressureForceField.cpp \
           forcefield/LennardJonesForceField.cpp \
           forcefield/PlaneForceField.cpp \
           forcefield/SphereForceField.cpp \
           forcefield/ConicalForceField.cpp \
           forcefield/EllipsoidForceField.cpp \
           forcefield/FrameSpringForceField.cpp \
           forcefield/SPHFluidForceField.cpp \
           forcefield/SpringForceField.cpp \
           forcefield/StiffSpringForceField.cpp \
           forcefield/JointSpringForceField.cpp \
           forcefield/PenalityContactForceField.cpp \
           forcefield/MeshSpringForceField.cpp \
           forcefield/QuadularBendingSprings.cpp \
           forcefield/QuadBendingSprings.cpp \
           forcefield/RegularGridSpringForceField.cpp \
           forcefield/TetrahedronFEMForceField.cpp \
           forcefield/HexahedronFEMForceField.cpp \
           forcefield/HexahedralFEMForceField.cpp \
           forcefield/TetrahedralTensorMassForceField.cpp \
           forcefield/TetrahedralCorotationalFEMForceField.cpp \
           forcefield/TriangleBendingSprings.cpp \
           forcefield/TriangularBendingSprings.cpp \
           forcefield/TriangleFEMForceField.cpp \
           forcefield/TriangularFEMForceField.cpp \
           forcefield/TriangularAnisotropicFEMForceField.cpp \
           forcefield/TrianglePressureForceField.cpp \
           forcefield/TriangularBiquadraticSpringsForceField.cpp \
           forcefield/TriangularQuadraticSpringsForceField.cpp \
           forcefield/TriangularTensorMassForceField.cpp \
           forcefield/VectorSpringForceField.cpp \
           interactionforcefield/RepulsiveSpringForceField.cpp \
           interactionforcefield/InteractionEllipsoidForceField.cpp \
           linearsolver/CGLinearSolver.cpp \
           linearsolver/LULinearSolver.cpp \
           linearsolver/BTDLinearSolver.cpp \
           mapping/ArticulatedSystemMapping.cpp \
           mapping/BarycentricMapping.cpp \
           mapping/BeamLinearMapping.cpp \
           mapping/CenterOfMassMapping.cpp \
           mapping/CurveMapping.cpp \
           mapping/IdentityMapping.cpp \
           mapping/ImplicitSurfaceMapping.cpp \
           mapping/LaparoscopicRigidMapping.cpp \
           mapping/RigidMapping.cpp \
           mapping/RigidRigidMapping.cpp \
           mapping/LineSetSkinningMapping.cpp \
           mapping/SkinningMapping.cpp \
           mapping/SPHFluidSurfaceMapping.cpp \
           mapping/SubsetMapping.cpp \
           mapping/TubularMapping.cpp \
           mapping/VoidMapping.cpp \
           mass/DiagonalMass.cpp \
           mass/MatrixMass.cpp \
           mass/UniformMass.cpp \
           misc/CompareState.cpp \
           misc/Monitor.cpp \
           misc/ParticleSink.cpp \
           misc/ParticleSource.cpp \
           misc/ReadState.cpp \
           misc/WriteState.cpp \
           mastersolver/DefaultMasterSolver.cpp \
           mastersolver/MultiStepMasterSolver.cpp \
           mastersolver/MasterContactSolver.cpp \
           odesolver/CentralDifferenceSolver.cpp \
           odesolver/CGImplicitSolver.cpp \
           odesolver/EulerSolver.cpp \
           odesolver/EulerImplicitSolver.cpp \
           odesolver/DampVelocitySolver.cpp \
           odesolver/RungeKutta2Solver.cpp \
           odesolver/RungeKutta4Solver.cpp \
           odesolver/StaticSolver.cpp \
           topology/CubeTopology.cpp \
           topology/CylinderGridTopology.cpp \
           topology/Edge2QuadTopologicalMapping.cpp \
           topology/EdgeSetGeometryAlgorithms.cpp \
           topology/EdgeSetTopologyAlgorithms.cpp \
           topology/EdgeSetTopologyContainer.cpp \
           topology/EdgeSetTopologyModifier.cpp \
           topology/GridTopology.cpp \
           topology/Hexa2QuadTopologicalMapping.cpp \
           topology/HexahedronSetGeometryAlgorithms.cpp \
           topology/HexahedronSetTopologyAlgorithms.cpp \
           topology/HexahedronSetTopologyContainer.cpp \
           topology/HexahedronSetTopologyModifier.cpp \
           topology/ManifoldEdgeSetGeometryAlgorithms.cpp \
           topology/ManifoldEdgeSetTopologyAlgorithms.cpp \
           topology/ManifoldEdgeSetTopologyContainer.cpp \
           topology/ManifoldEdgeSetTopologyModifier.cpp \
           topology/MeshTopology.cpp \
           topology/PointData.cpp \
           topology/PointSetGeometryAlgorithms.cpp \
           topology/PointSetTopologyAlgorithms.cpp \
           topology/PointSetTopologyContainer.cpp \
           topology/PointSetTopologyModifier.cpp \
           topology/PointSubset.cpp \
           topology/Quad2TriangleTopologicalMapping.cpp \
           topology/QuadSetGeometryAlgorithms.cpp \
           topology/QuadSetTopologyAlgorithms.cpp \
           topology/QuadSetTopologyContainer.cpp \
           topology/QuadSetTopologyModifier.cpp \
           topology/RegularGridTopology.cpp \
           topology/SparseGridTopology.cpp \
           topology/Tetra2TriangleTopologicalMapping.cpp \
           topology/TetrahedronSetGeometryAlgorithms.cpp \
           topology/TetrahedronSetTopologyAlgorithms.cpp \
           topology/TetrahedronSetTopologyContainer.cpp \
           topology/TetrahedronSetTopologyModifier.cpp \
           topology/Triangle2EdgeTopologicalMapping.cpp \
           topology/TriangleSetGeometryAlgorithms.cpp \
           topology/TriangleSetTopologyAlgorithms.cpp \
           topology/TriangleSetTopologyContainer.cpp \
           topology/TriangleSetTopologyModifier.cpp \
           visualmodel/ClipPlane.cpp \
           visualmodel/DrawV.cpp \
           visualmodel/OglModel.cpp \
           visualmodel/VisualModelImpl.cpp \
           visualmodel/Light.cpp \
           visualmodel/LightManager.cpp


contains(DEFINES,SOFA_HAVE_GLEW){

    HEADERS += \
           visualmodel/OglShader.h \
           visualmodel/OglTexture.h \
           visualmodel/OglVariable.h \
           visualmodel/OglShaderMacro.h \
           visualmodel/OglTetrahedralModel.h \
           visualmodel/OglTetrahedralModel.inl

    SOURCES += \
           visualmodel/OglShader.cpp \
           visualmodel/OglTexture.cpp \
           visualmodel/OglVariable.cpp \
           visualmodel/OglShaderMacro.cpp \
           visualmodel/OglTetrahedralModel.cpp

}


contains(DEFINES,SOFA_TEST_FRICTION){

    HEADERS += forcefield/PenalityContactFrictionForceField.h \
           forcefield/PenalityContactFrictionForceField.inl

    SOURCES += forcefield/PenalityContactFrictionForceField.cpp \

}

contains(DEFINES,SOFA_HAVE_MKL){

    HEADERS += \
	linearsolver/MKLMatrix.h \
	linearsolver/MKLVector.h

}

contains(DEFINES,SOFA_HAVE_SENSABLE){

    HEADERS +=  controller/OmniDriver.h \
           	controller/ForceFeedback.h \
           	controller/NullForceFeedback.h \
           	controller/EnslavementForceFeedback.h \
		controller/LCPForceFeedback.h

    SOURCES +=  controller/OmniDriver.cpp \
           	controller/NullForceFeedback.cpp \
           	controller/EnslavementForceFeedback.cpp \
		controller/LCPForceFeedback.cpp

}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(component-local.cfg) 
