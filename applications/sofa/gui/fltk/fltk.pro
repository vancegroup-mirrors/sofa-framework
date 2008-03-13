# Target is a library: sofaguifltk

SOFA_DIR = ../../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = sofaguifltk$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES
LIBS += $$SOFA_FRAMEWORK_LIBS $$SOFA_MODULES_LIBS
LIBS += -lsofagui$$LIBSUFFIX
LIBS += $$SOFA_EXT_LIBS

SOURCES = \
	    Callbacks.cpp \
	    FLTKviewer.cpp \
	    GUI.cpp \
	    Main.cpp

HEADERS = \
	    Callbacks.h \
	    FLTKviewer.h \
	    GUI.h \
	    Main.h 

