load(sofa/pre)

TEMPLATE = lib
TARGET = sofa_loader

DEFINES += SOFA_BUILD_LOADER

HEADERS += loader/MeshGmshLoader.h \
           loader/MeshObjLoader.h \
           loader/MeshOffLoader.h \
           loader/MeshTrianLoader.h \
           loader/MeshVTKLoader.h \
           loader/MeshSTLLoader.h \
           loader/MeshXspLoader.h \
           loader/OffSequenceLoader.h \
           misc/InputEventReader.h \
           misc/ReadState.h \
           misc/ReadState.inl \
           misc/ReadTopology.h \
           misc/ReadTopology.inl

SOURCES += loader/MeshGmshLoader.cpp \
           loader/MeshObjLoader.cpp \
           loader/MeshOffLoader.cpp \
           loader/MeshTrianLoader.cpp \
           loader/MeshVTKLoader.cpp \
           loader/MeshSTLLoader.cpp \
           loader/MeshXspLoader.cpp \
           loader/OffSequenceLoader.cpp \
           misc/InputEventReader.cpp \
           misc/ReadState.cpp \
           misc/ReadTopology.cpp

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_INSTALL_INC_DIR/applications

#exists(component-local.cfg): include(component-local.cfg)

load(sofa/post)
 