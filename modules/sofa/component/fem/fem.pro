SOFA_DIR=../../../..

TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

#NO dependencies
#SUBDIRS += fetype
#SUBDIRS += forcefield
#SUBDIRS += material
#SUBDIRS += straintensor
SUBDIRS += libfem.pro
