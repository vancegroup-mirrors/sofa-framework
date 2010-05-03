# Target is a library:  wiiuse

SOFA_DIR = ../..
TEMPLATE = lib
TARGET = wiiuse

include($${SOFA_DIR}/sofa.cfg)

CONFIG += $$CONFIGLIBRARIES

LIBS += -lbluetooth

HEADERS += \
	src/classic.h \
        src/dynamics.h \
        src/events.h \
        src/io.h \
        src/ir.h \
        src/nunchuk.h \
        src/guitar_hero_3.h \
        src/wiiuse.h

SOURCES += \
	src/classic.c \
	src/dynamics.c \
	src/events.c \
	src/io.c \
	src/ir.c \
	src/nunchuk.c \
	src/guitar_hero_3.c \
	src/wiiuse.c

win32 {
	SOURCES += src/io_win.c
}

unix {
	SOURCES += src/io_nix.c
}
