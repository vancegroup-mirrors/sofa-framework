SOFA_DIR = ../../../..
TEMPLATE = lib
TARGET = sofacomponentconfigurationsetting

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

!contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
CONFIG += dll
}

DEFINES += SOFA_BUILD_COMPONENT_CONFIGURATIONSETTING

HEADERS += initConfigurationSetting.h \
           AddFrameButtonSetting.h \
           AttachBodyButtonSetting.h \
           BackgroundSetting.h  \
           FixPickedParticleButtonSetting.h \
           MouseButtonSetting.h \
           SofaDefaultPathSetting.h \
           StatsSetting.h \
           ViewerSetting.h


SOURCES += initConfigurationSetting.cpp \
           AddFrameButtonSetting.cpp \
           AttachBodyButtonSetting.cpp \
           BackgroundSetting.cpp \
           FixPickedParticleButtonSetting.cpp \
           MouseButtonSetting.cpp \
           SofaDefaultPathSetting.cpp \
           StatsSetting.cpp \
           ViewerSetting.cpp


contains(DEFINES,SOFA_GUI_QTOGREVIEWER){

HEADERS += OgreViewerSetting.h

SOURCES += OgreViewerSetting.cpp
} 


LIBS += $$SOFA_FRAMEWORK_LIBS
LIBS += -lsofasimulation$$LIBSUFFIX
LIBS += -lsofatree$$LIBSUFFIX
LIBS += -lsofacomponentmastersolver$$LIBSUFFIX
LIBS += -lsofacomponentfem$$LIBSUFFIX
LIBS += -lsofacomponentinteractionforcefield$$LIBSUFFIX
LIBS += -lsofacomponentcontextobject$$LIBSUFFIX
LIBS += -lsofacomponentbehaviormodel$$LIBSUFFIX
LIBS += -lsofacomponentlinearsolver$$LIBSUFFIX
LIBS += -lsofacomponentodesolver$$LIBSUFFIX
LIBS += -lsofacomponentcontroller$$LIBSUFFIX
LIBS += -lsofacomponentvisualmodel$$LIBSUFFIX
LIBS += -lsofacomponentmass$$LIBSUFFIX
LIBS += -lsofacomponentforcefield$$LIBSUFFIX
LIBS += -lsofacomponentmapping$$LIBSUFFIX
LIBS += -lsofacomponentconstraint$$LIBSUFFIX
LIBS += -lsofacomponentcollision$$LIBSUFFIX
LIBS += -lsofacomponentbase$$LIBSUFFIX
LIBS += -lsofacomponentmisc$$LIBSUFFIX

LIBS += $$SOFA_EXT_LIBS 

macx : LIBS += -framework GLUT 

# Make sure there are no cross-dependencies
INCLUDEPATH -= $$SOFA_DIR/applications

include(configurationsetting-local.cfg) 
