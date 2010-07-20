SOFA_DIR=../../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += libbase.pro

#NO dependencies
SUBDIRS += loader
SUBDIRS += odesolver
SUBDIRS += forcefield
SUBDIRS += mass
SUBDIRS += fem
SUBDIRS += behaviormodel
SUBDIRS += visualmodel
SUBDIRS += contextobject

SUBDIRS += interactionforcefield #forcefield dependency
SUBDIRS += linearsolver          #forcefield + odesolver dependency

SUBDIRS += mapping               #forcefield + visualmodel dependency
SUBDIRS += constraint            #forcefield + odesolver + linearsolver + mass dependency
SUBDIRS += mastersolver          #linearsolver + constraint dependency
SUBDIRS += controller            #forcefield + constraint dependency
SUBDIRS += collision             #linearsolver+odesolver+forcefield+mapping+constraint+visualmodel
SUBDIRS += engine                #collision dependency
SUBDIRS += misc                  #full dependencies
SUBDIRS += configurationsetting  #full dependencies
SUBDIRS += libcomponent.pro

