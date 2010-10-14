SOFA_DIR = ../../..
TEMPLATE = lib
TARGET = sofacomponentbase

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_CONTAINER
DEFINES += SOFA_BUILD_COMPONENT_TOPOLOGY
#DEFINES += SOFA_BUILD_COMPONENT_FEM
DEFINES += POINT_DATA_VECTOR_ACCESS

HEADERS += component.h \
           container/MeshLoader.h \
		   container/MultiMeshLoader.h \
           container/MappedObject.h \
           container/MappedObject.inl \
           container/MechanicalObject.h \
           container/MechanicalObject.inl \
           container/VoxelGridLoader.h \
           container/ArticulatedHierarchyContainer.h \
           container/ArticulatedHierarchyContainer.inl \
           container/ImplicitSurfaceContainer.h \
           container/InterpolatedImplicitSurface.h \
           container/InterpolatedImplicitSurface.inl \
           container/RotationFinder.h \
           container/RotationFinder.inl \
           container/SpatialGridContainer.h \
           container/SpatialGridContainer.inl \   
           container/DistanceGrid.h \
           topology/CenterPointTopologicalMapping.h \
           topology/CommonAlgorithms.h \
           topology/CubeTopology.h \
           topology/CylinderGridTopology.h \
           topology/DynamicSparseGridGeometryAlgorithms.inl \
           topology/DynamicSparseGridGeometryAlgorithms.h \
           topology/DynamicSparseGridTopologyAlgorithms.inl \
           topology/DynamicSparseGridTopologyAlgorithms.h \
           topology/DynamicSparseGridTopologyContainer.h \
           topology/DynamicSparseGridTopologyModifier.h \
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
           topology/EdgeSetTopologyEngine.h \
	   topology/EdgeSetTopologyEngine.inl \
           topology/EdgeSubsetData.h \
           topology/EdgeSubsetData.inl \
           topology/GridTopology.h \
           topology/Hexa2QuadTopologicalMapping.h \
           topology/Hexa2TetraTopologicalMapping.h \
           topology/HexahedronData.h \
           topology/HexahedronData.inl \
           topology/HexahedronSetGeometryAlgorithms.h \
           topology/HexahedronSetTopologyAlgorithms.h \
           topology/HexahedronSetTopologyChange.h \
           topology/HexahedronSetTopologyContainer.h \
           topology/HexahedronSetTopologyModifier.h \
           topology/HexahedronSetGeometryAlgorithms.inl \
           topology/HexahedronSetTopologyAlgorithms.inl \
           topology/HexahedronSetTopologyEngine.h \
           topology/HexahedronSetTopologyEngine.inl \ 
           topology/ManifoldEdgeSetGeometryAlgorithms.h \
           topology/ManifoldEdgeSetTopologyAlgorithms.h \
           topology/ManifoldEdgeSetTopologyContainer.h \
           topology/ManifoldEdgeSetTopologyModifier.h \
           topology/ManifoldEdgeSetGeometryAlgorithms.inl \
           topology/ManifoldEdgeSetTopologyAlgorithms.inl \
           topology/ManifoldTriangleSetTopologyContainer.h \
           topology/ManifoldTriangleSetTopologyModifier.h \
           topology/ManifoldTriangleSetTopologyAlgorithms.h \
           topology/ManifoldTriangleSetTopologyAlgorithms.inl \
           topology/ManifoldTetrahedronSetTopologyContainer.h \
           topology/MeshTopology.h \
           topology/Mesh2PointTopologicalMapping.h \
           topology/MultilevelHexahedronSetTopologyContainer.h \
           topology/PointData.h \
           topology/PointData.inl \
           topology/PointSetTopologyEngine.h \
           topology/PointSetTopologyEngine.inl \
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
           topology/QuadSetTopologyEngine.h \
           topology/QuadSetTopologyEngine.inl \
           topology/RegularGridTopology.h \
           topology/SimpleTesselatedHexaTopologicalMapping.h \
           topology/SimpleTesselatedTetraTopologicalMapping.h \
           topology/SparseGridMultipleTopology.h \
           topology/SparseGridRamificationTopology.h  \
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
           topology/TetrahedronSetTopologyEngine.h \
           topology/TetrahedronSetTopologyEngine.inl \
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
           topology/TriangleSetTopologyEngine.h \
           topology/TriangleSetTopologyEngine.inl \  
           topology/TriangleSubsetData.h \
           topology/TriangleSubsetData.inl 

SOURCES +=  \
           container/initContainer.cpp \
           container/MeshLoader.cpp \
		   container/MultiMeshLoader.cpp \
           container/MappedObject.cpp \
           container/MechanicalObject.cpp \
           container/VoxelGridLoader.cpp \
           container/ArticulatedHierarchyContainer.cpp \
           container/ImplicitSurfaceContainer.cpp \
           container/InterpolatedImplicitSurface.cpp \
           container/RotationFinder.cpp \
           container/SpatialGridContainer.cpp \
           container/DistanceGrid.cpp \
           topology/initTopology.cpp \
           topology/CenterPointTopologicalMapping.cpp \
           topology/CubeTopology.cpp \
           topology/CylinderGridTopology.cpp \
           topology/DynamicSparseGridGeometryAlgorithms.cpp \
           topology/DynamicSparseGridTopologyAlgorithms.cpp \
           topology/DynamicSparseGridTopologyContainer.cpp \
           topology/DynamicSparseGridTopologyModifier.cpp \
           topology/Edge2QuadTopologicalMapping.cpp \
           topology/EdgeSetGeometryAlgorithms.cpp \
           topology/EdgeSetTopologyAlgorithms.cpp \
           topology/EdgeSetTopologyContainer.cpp \
           topology/EdgeSetTopologyModifier.cpp \
           topology/GridTopology.cpp \
           topology/Hexa2QuadTopologicalMapping.cpp \
           topology/Hexa2TetraTopologicalMapping.cpp \
           topology/HexahedronSetGeometryAlgorithms.cpp \
           topology/HexahedronSetTopologyAlgorithms.cpp \
           topology/HexahedronSetTopologyContainer.cpp \
           topology/HexahedronSetTopologyModifier.cpp \
           topology/ManifoldEdgeSetGeometryAlgorithms.cpp \
           topology/ManifoldEdgeSetTopologyAlgorithms.cpp \
           topology/ManifoldEdgeSetTopologyContainer.cpp \
           topology/ManifoldEdgeSetTopologyModifier.cpp \
           topology/ManifoldTriangleSetTopologyContainer.cpp \
           topology/ManifoldTriangleSetTopologyModifier.cpp \
           topology/ManifoldTriangleSetTopologyAlgorithms.cpp \
           topology/ManifoldTetrahedronSetTopologyContainer.cpp \
           topology/MeshTopology.cpp \
           topology/Mesh2PointTopologicalMapping.cpp \
           topology/MultilevelHexahedronSetTopologyContainer.cpp \
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
           topology/SimpleTesselatedHexaTopologicalMapping.cpp \
           topology/SimpleTesselatedTetraTopologicalMapping.cpp \
           topology/SparseGridMultipleTopology.cpp \
           topology/SparseGridRamificationTopology.cpp   \
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



contains(DEFINES,SOFA_SMP){
HEADERS += \
           container/MechanicalObjectTasks.inl
}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(libbase-local.cfg): include(libbase-local.cfg) 
