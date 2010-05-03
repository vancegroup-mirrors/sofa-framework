# Target is a library:  tinyxml

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = vrpn_server$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

QUATINCLUDES = ../quat
ATMELINCLUDES = atmellib/

LIBS += -ldl -lvrpn_quat -lvrpn_atmellib -lXext -lX11 -lm
contains(DEFINES,VRPN_USE_WIIUSE){
	LIBS += -lwiiuse
}
INCLUDEPATH += $$QUATINCLUDES $$ATMELINCLUDES

DEFINES += linux

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
	vrpn_GlobalHapticsOrb.C vrpn_Tracker_ButtonFly.h vrpn_ADBox.h \
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
	vrpn_Connection.C vrpn_Tracker.C vrpn_Button.C \
	vrpn_ForceDevice.C vrpn_Shared.C \
	vrpn_Analog.C vrpn_FileConnection.C \
	vrpn_FileController.C vrpn_Forwarder.C vrpn_Text.C \
	vrpn_ForwarderController.C vrpn_Serial.C vrpn_Dial.C \
	vrpn_SharedObject.C vrpn_BaseClass.C \
	vrpn_Sound.C vrpn_LamportClock.C vrpn_Mutex.C \
	vrpn_RedundantTransmission.C vrpn_Imager.C \
	vrpn_Analog_Output.C vrpn_Poser.C vrpn_Auxiliary_Logger.C \
	vrpn_3Space.C \
	vrpn_Flock.C vrpn_Tracker_Fastrak.C vrpn_Dyna.C \
	vrpn_Flock_Parallel.C  vrpn_UNC_Joystick.C \
	vrpn_JoyFly.C vrpn_sgibox.C vrpn_CerealBox.C \
	vrpn_Tracker_AnalogFly.C vrpn_raw_sgibox.C vrpn_Magellan.C \
	vrpn_Analog_Radamec_SPI.C vrpn_ImmersionBox.C vrpn_Wanda.C \
	vrpn_Analog_5dt.C vrpn_Joylin.C vrpn_Tng3.C vrpn_Spaceball.C \
	vrpn_Tracker_isense.C vrpn_Zaber.C vrpn_nikon_controls.C \
	vrpn_GlobalHapticsOrb.C vrpn_Tracker_ButtonFly.C vrpn_ADBox.C \
	vrpn_VPJoystick.C vrpn_Tracker_Liberty.C vrpn_NationalInstruments.C \
	vrpn_Poser_Analog.C vrpn_Tracker_DTrack.C vrpn_Poser_Tek4662.C \
	vrpn_Tracker_Crossbow.C vrpn_Tracker_3DMouse.C \
	vrpn_Mouse.C vrpn_3DMicroscribe.C vrpn_5DT16.C \
	vrpn_ForceDeviceServer.C vrpn_Keyboard.C \
	vrpn_Analog_USDigital_A2.C vrpn_Button_NI_DIO24.C \
	vrpn_Tracker_PhaseSpace.C \
	vrpn_Atmel.C vrpn_inertiamouse.C vrpn_Event.C vrpn_Event_Analog.C \
	vrpn_Event_Mouse.C vrpn_Imager_Stream_Buffer.C \
	vrpn_HumanInterface.C vrpn_Xkeys.C vrpn_3DConnexion.C \
	vrpn_Tracker_MotionNode.C vrpn_Tracker_NDI_Polaris.C \
	vrpn_WiiMote.C

