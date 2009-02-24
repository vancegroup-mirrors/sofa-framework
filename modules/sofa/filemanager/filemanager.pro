SOFA_DIR=../../..
TEMPLATE = subdirs
include($${SOFA_DIR}/sofa.cfg)

contains (DEFINES, SOFA_PML) {
	SUBDIRS += sofapml
}
