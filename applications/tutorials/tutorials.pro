SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += oneParticule
SUBDIRS += oneTetrahedron
SUBDIRS += mixedPendulum
        
        