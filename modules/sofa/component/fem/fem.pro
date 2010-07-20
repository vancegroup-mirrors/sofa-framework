SOFA_DIR=../../../..

TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

#NO dependencies
#SUBDIRS += fetype
#SUBDIRS += straintensor
#SUBDIRS += material

SUBDIRS += libfem.pro

#SUBDIRS += forcefield
