SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)
CONFIG -= ordered


SUBDIRS += PluginExample

contains (DEFINES, SOFA_HAVE_ARTRACK) {
SUBDIRS += ARTrack
}


!include(plugins-local.cfg) {
}
