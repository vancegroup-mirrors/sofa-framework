SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentinteractionforcefield

include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_INTERACTIONFORCEFIELD

HEADERS += \
		initInteractionForceField.h        \
		BoxStiffSpringForceField.h         \
		BoxStiffSpringForceField.inl       \
		FrameSpringForceField.h            \
		FrameSpringForceField.inl          \
		InteractionEllipsoidForceField.h   \
		InteractionEllipsoidForceField.inl \
		JointSpringForceField.h            \
		JointSpringForceField.inl          \
		MeshSpringForceField.h             \
		MeshSpringForceField.inl           \
		PenalityContactForceField.h        \
		PenalityContactForceField.inl      \
		QuadBendingSprings.h               \
		QuadBendingSprings.inl             \
		RegularGridSpringForceField.h      \
		RegularGridSpringForceField.inl    \
		RepulsiveSpringForceField.h        \
		RepulsiveSpringForceField.inl      \
#	SparseGridSpringForceField.h           \
#	SparseGridSpringForceField.inl         \
		SpringForceField.h                 \
		SpringForceField.inl               \
		StiffSpringForceField.h            \
		StiffSpringForceField.inl          \
		TriangleBendingSprings.h           \
		TriangleBendingSprings.inl         \
		VectorSpringForceField.h           \
		VectorSpringForceField.inl
           
SOURCES += \
		initInteractionForceField.cpp      \
		BoxStiffSpringForceField.cpp       \
		FrameSpringForceField.cpp          \
		InteractionEllipsoidForceField.cpp \
		JointSpringForceField.cpp          \
		MeshSpringForceField.cpp           \
		PenalityContactForceField.cpp      \
		QuadBendingSprings.cpp             \
		RegularGridSpringForceField.cpp    \
		RepulsiveSpringForceField.cpp      \
#	SparseGridSpringForceField.cpp         \
		SpringForceField.cpp               \
		StiffSpringForceField.cpp          \
		TriangleBendingSprings.cpp         \
		VectorSpringForceField.cpp       







LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

exists(interactionforcefield-local.cfg): include(interactionforcefield-local.cfg) 
