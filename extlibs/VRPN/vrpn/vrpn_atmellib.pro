# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = vrpn_atmellib$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat

LIBS += -ldl -lvrpn_quat -lXext -lX11 -lm
INCLUDEPATH += $$QUATINCLUDES

HEADERS += \
	vrpn_atmellib.h vrpn_atmellib_helper.h vrpn_atmellib_errno.h

SOURCES += \
	atmellib/vrpn_atmellib_iobasic.C atmellib/vrpn_atmellib_helper.C \
	atmellib/vrpn_atmellib_openclose.C atmellib/vrpn_atmellib_register.C atmellib/vrpn_atmellib_tester.C

