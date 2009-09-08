SOFA_DIR=../../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += libbase.pro


SUBDIRS += forcefield
SUBDIRS += linearsolver
SUBDIRS += mastersolver
SUBDIRS += fem
SUBDIRS += contextobject
SUBDIRS += behaviormodel
SUBDIRS += odesolver
SUBDIRS += visualmodel
SUBDIRS += mass
SUBDIRS += interactionforcefield
SUBDIRS += controller
SUBDIRS += mapping
SUBDIRS += constraint
SUBDIRS += collision
SUBDIRS += misc
SUBDIRS += engine
SUBDIRS += libcomponent.pro

