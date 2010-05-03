# Target is a library:  VRPN

SOFA_DIR = ../..
TEMPLATE = subdirs
include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += quat
SUBDIRS += vrpn

