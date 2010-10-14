# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
TARGET = vrpn_server

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat
ATMELINCLUDES = atmellib/

LIBS += -ldl -lvrpn_quat$$LIBSUFFIX -lvrpn_atmellib$$LIBSUFFIX
contains(DEFINES,VRPN_USE_WIIUSE){
	LIBS += -lwiiuse
}
INCLUDEPATH += $$QUATINCLUDES $$ATMELINCLUDES

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
	vrpn_Analog_Output.h vrpn_Poser.h vrpn_Auxiliary_Logger.h \
	vrpn_3Space.h \
	vrpn_Flock.h vrpn_Tracker_Fastrak.h vrpn_Dyna.h \
	vrpn_Flock_Parallel.h vrpn_UNC_Joystick.h \
	vrpn_JoyFly.h vrpn_sgibox.h vrpn_raw_sgibox.h \
	vrpn_CerealBox.h vrpn_Tracker_AnalogFly.h vrpn_Magellan.h \
	vrpn_Analog_Radamec_SPI.h vrpn_ImmersionBox.h vrpn_Wanda.h \
	vrpn_Analog_5dt.h vrpn_Joylin.h vrpn_Tng3.h vrpn_Spaceball.h \
	vrpn_tracker_isense.h vrpn_Zaber.h vrpn_nikon_controls.h \
	vrpn_GlobalHapticsOrb.cpp vrpn_Tracker_ButtonFly.h vrpn_ADBox.h \
	vrpn_VPJoystick.h vrpn_Tracker_Liberty.h vrpn_NationalInstruments.h \
	vrpn_Poser_Analog.h vrpn_Tracker_DTrack.h vrpn_Poser.h \
	vrpn_Poser_Tek4662.h vrpn_Tracker_Crossbow.h vrpn_Tracker_3DMouse.h \
	vrpn_Mouse.h vrpn_3DMicroscribe.h vrpn_5DT16.h \
	vrpn_ForceDeviceServer.h vrpn_Keyboard.h \
	vrpn_Analog_USDigital_A2.h vrpn_Button_NI_DIO24.h \
	vrpn_Tracker_PhaseSpace.h vrpn_Atmel.h \
	vrpn_inertiamouse.h vrpn_Event.h vrpn_Event_Analog.h \
	vrpn_Event_Mouse.h vrpn_Imager_Stream_Buffer.h \
	vrpn_HumanInterface.h vrpn_Xkeys.h vrpn_3DConnexion.h \
	vrpn_Tracker_MotionNode.h vrpn_Tracker_NDI_Polaris.h \
	vrpn_WiiMote.h
	

SOURCES += \
	vrpn_Connection.cpp vrpn_Tracker.cpp vrpn_Button.cpp \
	vrpn_ForceDevice.cpp vrpn_Shared.cpp \
	vrpn_Analog.cpp vrpn_FileConnection.cpp \
	vrpn_FileController.cpp vrpn_Forwarder.cpp vrpn_Text.cpp \
	vrpn_ForwarderController.cpp vrpn_Serial.cpp vrpn_Dial.cpp \
	vrpn_SharedObject.cpp vrpn_BaseClass.cpp \
	vrpn_Sound.cpp vrpn_LamportClock.cpp vrpn_Mutex.cpp \
	vrpn_RedundantTransmission.cpp vrpn_Imager.cpp \
	vrpn_Analog_Output.cpp vrpn_Poser.cpp vrpn_Auxiliary_Logger.cpp \
	vrpn_3Space.cpp \
	vrpn_Flock.cpp vrpn_Tracker_Fastrak.cpp vrpn_Dyna.cpp \
	vrpn_Flock_Parallel.cpp  vrpn_UNC_Joystick.cpp \
	vrpn_JoyFly.cpp vrpn_sgibox.cpp vrpn_CerealBox.cpp \
	vrpn_Tracker_AnalogFly.cpp vrpn_raw_sgibox.cpp vrpn_Magellan.cpp \
	vrpn_Analog_Radamec_SPI.cpp vrpn_ImmersionBox.cpp vrpn_Wanda.cpp \
	vrpn_Analog_5dt.cpp vrpn_Joylin.cpp vrpn_Tng3.cpp vrpn_Spaceball.cpp \
	vrpn_Tracker_isense.cpp vrpn_Zaber.cpp vrpn_nikon_controls.cpp \
	vrpn_GlobalHapticsOrb.cpp vrpn_Tracker_ButtonFly.cpp vrpn_ADBox.cpp \
	vrpn_VPJoystick.cpp vrpn_Tracker_Liberty.cpp vrpn_NationalInstruments.cpp \
	vrpn_Poser_Analog.cpp vrpn_Tracker_DTrack.cpp vrpn_Poser_Tek4662.cpp \
	vrpn_Tracker_Crossbow.cpp vrpn_Tracker_3DMouse.cpp \
	vrpn_Mouse.cpp vrpn_3DMicroscribe.cpp vrpn_5DT16.cpp \
	vrpn_ForceDeviceServer.cpp vrpn_Keyboard.cpp \
	vrpn_Analog_USDigital_A2.cpp vrpn_Button_NI_DIO24.cpp \
	vrpn_Tracker_PhaseSpace.cpp \
	vrpn_Atmel.cpp vrpn_inertiamouse.cpp vrpn_Event.cpp vrpn_Event_Analog.cpp \
	vrpn_Event_Mouse.cpp vrpn_Imager_Stream_Buffer.cpp \
	vrpn_HumanInterface.cpp vrpn_Xkeys.cpp vrpn_3DConnexion.cpp \
	vrpn_Tracker_MotionNode.cpp vrpn_Tracker_NDI_Polaris.cpp \
	vrpn_WiiMote.cpp

