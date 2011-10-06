load(sofa/pre)

TEMPLATE = lib
TARGET = sofa_misc_engine

DEFINES += SOFA_BUILD_MISC_ENGINE

HEADERS += engine/Distances.h \
           engine/Distances.inl

SOURCES += engine/Distances.cpp

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_INSTALL_INC_DIR/applications

#exists(component-local.cfg): include(component-local.cfg)

load(sofa/post)
 