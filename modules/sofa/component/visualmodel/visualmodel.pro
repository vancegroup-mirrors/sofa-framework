SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentvisualmodel
include($${SOFA_DIR}/sofa.cfg)
CONFIG += $$CONFIGLIBRARIES
!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}
DEFINES += SOFA_BUILD_COMPONENT_VISUALMODEL
HEADERS += initVisualModel.h \
    ClipPlane.h \
    DrawV.h \
    OglModel.h \
    OglViewport.h \
    VisualModelImpl.h \
    Light.h \
    LightManager.h \
    PointSplatModel.h \
    OglRenderingSRGB.h \
    BaseCamera.h \
    InteractiveCamera.h
    
SOURCES += initVisualModel.cpp \
    ClipPlane.cpp \
    DrawV.cpp \
    OglModel.cpp \
    OglViewport.cpp \
    VisualModelImpl.cpp \
    Light.cpp \
    LightManager.cpp \
    PointSplatModel.cpp \
    OglRenderingSRGB.cpp \
    BaseCamera.cpp \
    InteractiveCamera.cpp
    
contains(DEFINES,SOFA_HAVE_GLEW) { 
    HEADERS += OglAttribute.h \
        OglAttribute.inl \
        OglShader.h \
        OglShaderMacro.h \
        OglShaderVisualModel.h \
        OglShadowShader.h \
        OglTetrahedralModel.h \
        OglTetrahedralModel.inl \
        OglTexture.h \
        OglVariable.h \
        OglVariable.inl \
        PostProcessManager.h \
        SlicedVolumetricModel.h
    SOURCES += OglAttribute.cpp \
        OglShader.cpp \
        OglShaderMacro.cpp \
        OglShaderVisualModel.cpp \
        OglShadowShader.cpp \
        OglTetrahedralModel.cpp \
        OglTexture.cpp \
        OglVariable.cpp \
        PostProcessManager.cpp \
        SlicedVolumetricModel.cpp
}

LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS
macx:LIBS += -framework GLUT

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications
include(visualmodel-local.cfg)
