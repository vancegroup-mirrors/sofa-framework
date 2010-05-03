# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = vrpn_quat$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

HEADERS += \
	quat.h

SOURCES += \
	quat.c \
	matrix.c \
	vector.c \
	xyzquat.c

