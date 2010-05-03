# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = vrpn_client$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat

LIBS += -ldl -lvrpn_quat -lXext -lX11 -lm
contains(DEFINES,VRPN_USE_WIIUSE){
	LIBS += -lwiiuse
}
INCLUDEPATH += $$QUATINCLUDES

DEFINES += linux

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
	vrpn_Connection.C vrpn_Tracker.C vrpn_Button.C \
	vrpn_ForceDevice.C vrpn_Shared.C \
	vrpn_Analog.C vrpn_FileConnection.C \
	vrpn_FileController.C vrpn_Forwarder.C vrpn_Text.C \
	vrpn_ForwarderController.C vrpn_Serial.C vrpn_Dial.C \
	vrpn_SharedObject.C vrpn_BaseClass.C \
	vrpn_Sound.C vrpn_LamportClock.C vrpn_Mutex.C \
	vrpn_RedundantTransmission.C vrpn_Imager.C \
	vrpn_Analog_Output.C vrpn_Poser.C vrpn_Auxiliary_Logger.C

