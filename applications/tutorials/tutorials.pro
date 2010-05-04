SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS -= ordered
SUBDIRS += objectCreator
SUBDIRS += oneParticule
SUBDIRS += oneTetrahedron
SUBDIRS += mixedPendulum
SUBDIRS += chainHybrid

!include(tutorials-local.cfg) {
}
