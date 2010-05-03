SOFA_DIR=../../..
TEMPLATE = app
TARGET = sensAble

include($${SOFA_DIR}/sofa.cfg)

DESTDIR = $$SOFA_DIR/bin
CONFIG += $$CONFIGPROJECTGUI 
LIBS += $$SOFA_GUI_LIBS
LIBS += $$SOFA_LIBS

SOURCES = Main.cpp SensAble.cpp
HEADERS = SensAble.h
