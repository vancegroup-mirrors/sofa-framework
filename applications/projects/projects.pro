SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)
CONFIG -= ordered

SUBDIRS += runSofa
SUBDIRS += meshconv
SUBDIRS += generateDoc
SUBDIRS += GenerateRigid
SUBDIRS += generateTypedefs

 
SUBDIRS += Modeler
!contains (DEFINES, SOFA_GUI_QTVIEWER) {
	!contains (DEFINES, SOFA_GUI_QGLVIEWER) {
		!contains (DEFINES, SOFA_GUI_QTOGREVIEWER) {
			SUBDIRS -= Modeler
		}
	}
}

contains (DEFINES, SOFA_GPU_CUDA) {
	SUBDIRS += sofaCUDA
}

contains(DEFINES, SOFA_HAVE_FLOWVR){
	SUBDIRS += SofaFlowVR
}

contains(DEFINES, SOFA_HAVE_SENSABLE){
	SUBDIRS += SensAble
}

contains(DEFINES, SOFA_HAVE_CHAI3D){
	SUBDIRS += Haptic
}


!include(projects-local.cfg) {
}
