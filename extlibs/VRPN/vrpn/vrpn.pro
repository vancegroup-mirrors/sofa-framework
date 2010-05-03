# Target is a library:  VRPN

SOFA_DIR = ../../..
TEMPLATE = subdirs
include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += vrpn_client.pro
SUBDIRS += vrpn_atmellib.pro
SUBDIRS += vrpn_server.pro

