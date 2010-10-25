SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)
CONFIG -= ordered


SUBDIRS += PluginExample

contains (DEFINES, SOFA_HAVE_ARTRACK) {
SUBDIRS += ARTrack
}

contains (DEFINES, SOFA_HAVE_SENSABLE) {
SUBDIRS += Sensable
}

contains (DEFINES, SOFA_HAVE_XITACT) {
SUBDIRS += Xitact
}


!include(plugins-local.cfg) {
}
