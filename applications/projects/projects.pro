SOFA_DIR=../..
TEMPLATE = subdirs

include($${SOFA_DIR}/sofa.cfg)

SUBDIRS += runSofa

SUBDIRS += GenerateRigid

SUBDIRS += generateDoc

SUBDIRS += meshconv

#SUBDIRS += sofaOGRE

contains (DEFINES, SOFA_GPU_CUDA) {
	SUBDIRS += sofaCUDA
}

contains(DEFINES, SOFA_HAVE_FLOWVR){
	SUBDIRS += SofaFlowVR
}
