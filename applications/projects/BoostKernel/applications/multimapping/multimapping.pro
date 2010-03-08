SOFA_DIR=../../../../..
TEMPLATE = app
TARGET = multimapping

include($${SOFA_DIR}/sofa.cfg)

#Uncomment if you are compiling the boost kernel

DESTDIR = $$SOFA_DIR/bin
CONFIG += $$CONFIGPROJECTGUI
LIBS += $$SOFA_GUI_LIBS
LIBS += $$SOFA_LIBS

# The following is a workaround to get KDevelop to detect the name of the program to start
unix {
QMAKE_POST_LINK = ln -sf multimapping$$SUFFIX $$DESTDIR/multimapping-latest
}


SOURCES = Main.cpp 
HEADERS = 
