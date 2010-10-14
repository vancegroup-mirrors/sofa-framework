# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
TARGET = vrpn_atmellib

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat

LIBS += -ldl -lvrpn_quat$$LIBSUFFIX
INCLUDEPATH += $$QUATINCLUDES


unix {
	!macx {
		LIBS += -lXext -lX11 -lm
	}
}

HEADERS += \
	vrpn_atmellib.h vrpn_atmellib_helper.h vrpn_atmellib_errno.h

SOURCES += \
	atmellib/vrpn_atmellib_iobasic.C atmellib/vrpn_atmellib_helper.C \
	atmellib/vrpn_atmellib_openclose.C atmellib/vrpn_atmellib_register.C atmellib/vrpn_atmellib_tester.C

