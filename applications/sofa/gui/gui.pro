SOFA_DIR=../../..
TEMPLATE = subdirs
include($${SOFA_DIR}/sofa.cfg)

# Base GUI library
SUBDIRS += libgui

# Add all possible subdirectories so that KDevelop can index all classes
SUBDIRS += glut
SUBDIRS += fltk
SUBDIRS += qt

# Remove subdirectories if they are not included in the configuration
!contains (DEFINES, SOFA_GUI_GLUT) {
	SUBDIRS -= glut
}

!contains (DEFINES, SOFA_GUI_FLTK) {
	SUBDIRS -= fltk
}

!contains (DEFINES, SOFA_GUI_QTVIEWER) {
	!contains (DEFINES, SOFA_GUI_QGLVIEWER) {
		!contains (DEFINES, SOFA_GUI_QTOGREVIEWER) {
			SUBDIRS -= qt
		}
	}
}

# Main GUI library
SUBDIRS += libguimain
