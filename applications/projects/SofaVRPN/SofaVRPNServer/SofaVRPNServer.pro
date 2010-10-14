SOFA_DIR=../../../..
TEMPLATE = app
TARGET = SofaVRPNServer

include($${SOFA_DIR}/sofa.cfg)

DESTDIR = $$SOFA_DIR/bin

DEPENDPATH += .
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/vrpn
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/vrpn/server_src
INCLUDEPATH += $$SOFA_DIR/extlibs/VRPN/quat

LIBS += -lvrpn_server$$LIBSUFFIX -lvrpn_quat$$LIBSUFFIX -lvrpn_atmellib$$LIBSUFFIX

SOURCES += 	Main.cpp \
			vrpn_Generic_server_object.cpp
