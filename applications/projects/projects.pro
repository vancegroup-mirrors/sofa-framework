SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)
CONFIG -= ordered

SUBDIRS += runSofa

SUBDIRS += GenerateRigid

SUBDIRS += generateDoc

SUBDIRS += meshconv

#SUBDIRS += sofaOGRE

#SUBDIRS += softArticulations

contains (DEFINES, SOFA_GPU_CUDA) {
	SUBDIRS += sofaCUDA
}

contains(DEFINES, SOFA_HAVE_FLOWVR){
	SUBDIRS += SofaFlowVR
}

contains(DEFINES, SOFA_HAVE_SENSABLE){
	SUBDIRS += SensAble
} 

!include(projects-local.cfg) {
}
