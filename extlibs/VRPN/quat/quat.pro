# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
TARGET = vrpn_quat

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

HEADERS += \
	quat.h

SOURCES += \
	quat.c \
	matrix.c \
	vector.c \
	xyzquat.c

