include(../../../../sofa.cfg)
TEMPLATE = subdirs
# SUBDIRS += ArticulatedSolid
# SUBDIRS += FEMcontact
# SUBDIRS += Fluid3DVortexParticles
# SUBDIRS += CUDA

# ArticulatedSolid requires QT
!contains(DEFINES, SOFA_GUI_QT) {
	message(Defines is $$DEFINES)
	SUBDIRS -= ArticulatedSolid
	SUBDIRS -= Fluid3DVortexParticles
}
