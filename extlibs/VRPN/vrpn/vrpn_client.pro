# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
TARGET = vrpn_client

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat

LIBS += -ldl -lvrpn_quat$$LIBSUFFIX
contains(DEFINES,VRPN_USE_WIIUSE){
	LIBS += -lwiiuse
}
INCLUDEPATH += $$QUATINCLUDES

linux { 
	DEFINES += linux
	LIBS +=  -lXext -lX11 -lm
}

macx: DEFINES += MACOSX


HEADERS += \
	vrpn_Connection.h vrpn_Tracker.h vrpn_Button.h \
	vrpn_Sound.h vrpn_ForceDevice.h vrpn_Shared.h \
	vrpn_Analog.h vrpn_FileConnection.h \
	vrpn_FileController.h vrpn_Forwarder.h vrpn_Text.h \
	vrpn_ForwarderController.h vrpn_Serial.h vrpn_Dial.h \
	vrpn_SharedObject.h vrpn_LamportClock.h vrpn_Mutex.h \
	vrpn_BaseClass.h vrpn_Imager.h \
	vrpn_Analog_Output.h vrpn_Poser.h vrpn_Auxiliary_Logger.h

SOURCES += \
	vrpn_Connection.cpp vrpn_Tracker.cpp vrpn_Button.cpp \
	vrpn_ForceDevice.cpp vrpn_Shared.cpp \
	vrpn_Analog.cpp vrpn_FileConnection.cpp \
	vrpn_FileController.cpp vrpn_Forwarder.cpp vrpn_Text.cpp \
	vrpn_ForwarderController.cpp vrpn_Serial.cpp vrpn_Dial.cpp \
	vrpn_SharedObject.cpp vrpn_BaseClass.cpp \
	vrpn_Sound.cpp vrpn_LamportClock.cpp vrpn_Mutex.cpp \
	vrpn_RedundantTransmission.cpp vrpn_Imager.cpp \
	vrpn_Analog_Output.cpp vrpn_Poser.cpp vrpn_Auxiliary_Logger.cpp

