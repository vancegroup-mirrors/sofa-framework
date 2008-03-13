# Target is a library:  NewMAT

SOFA_DIR = ../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = SLC$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

DEFINES += PCUBE

HEADERS += bfastUtil.h \
           bfastVector.h \
           distTree.h \
           slcSurface.h

SOURCES += bfastUtil.cpp \
           bfastVector.cpp \
           distTree.cpp \
           slcSurface.cpp \
           pcube/fpcube.c \
           pcube/pcube.c
