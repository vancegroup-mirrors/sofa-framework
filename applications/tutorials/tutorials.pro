SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS -= ordered
SUBDIRS += objectCreator
SUBDIRS += oneParticule
SUBDIRS += oneTetrahedron
SUBDIRS += mixedPendulum
SUBDIRS += chainHybrid


contains (DEFINES, SOFA_HAVE_EIGEN2) {	
SUBDIRS += houseOfCards
}

exists(tutorials-local.cfg): include(tutorials-local.cfg)
