SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)
CONFIG -= ordered

SUBDIRS += runSofa
SUBDIRS += meshconv
SUBDIRS += generateDoc
SUBDIRS += GenerateRigid
SUBDIRS += generateTypedefs

#Projects using Qt4: if no QtViewer is available, we deactivate their compilation
contains (DEFINES, SOFA_GUI_QTVIEWER) || contains (DEFINES, SOFA_GUI_QGLVIEWER) || contains (DEFINES, SOFA_GUI_QTOGREVIEWER) {
    SUBDIRS += Modeler
#    SUBDIRS += sofaConfiguration
}

contains (DEFINES, SOFA_GPU_CUDA) {
	SUBDIRS += sofaCUDA
}

contains (DEFINES, SOFA_GPU_OPENCL) {
	SUBDIRS += sofaOPENCL
}

contains(DEFINES, SOFA_HAVE_FLOWVR){
	SUBDIRS += SofaFlowVR
}

contains(DEFINES, SOFA_HAVE_CHAI3D){
	SUBDIRS += Haptic
}


exists(projects-local.cfg): include(projects-local.cfg)
