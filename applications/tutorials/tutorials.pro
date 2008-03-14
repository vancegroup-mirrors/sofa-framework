SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS -= ordered

SUBDIRS += oneParticule
SUBDIRS += oneTetrahedron
SUBDIRS += mixedPendulum

!include(tutorials-local.cfg) {
}
