HEADERS += ChaiDevice.h HapticMapping.h HapticMapping.inl

SOURCES += ChaiDevice.cpp HapticMapping.cpp

SOFA_DIR = ../../../..

TEMPLATE = lib

include($${SOFA_DIR}/sofa.cfg)

TARGET = chaiIntegration$$LIBSUFFIX

CONFIG += $$CONFIGLIBRARIES

LIBS += $$SOFA_EXT_LIBS

INCLUDEPATH += $$SOFA_EXT_LIBS


unix{
	DEFINES += _POSIX _MAX_PATH=260 _LINUX LINUX
}
