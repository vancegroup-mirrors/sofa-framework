SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += simulation
SUBDIRS += component
SUBDIRS += gpu
SUBDIRS += filemanager
