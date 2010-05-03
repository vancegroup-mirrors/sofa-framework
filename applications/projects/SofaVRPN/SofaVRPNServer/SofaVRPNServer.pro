
SOFA_DIR=../../../..
TEMPLATE = app


include($${SOFA_DIR}/sofa.cfg)

TARGET = SofaVRPNServer$$SUFFIX
DESTDIR = $$SOFA_DIR/bin

DEPENDPATH += .
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/vrpn
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/vrpn/server_src
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/quat

LIBS += -lvrpn_server

SOURCES += 	Main.cpp \
			vrpn_Generic_server_object.cpp
