include (../../sofa.cfg)
TEMPLATE = subdirs
SUBDIRS += example1
SUBDIRS += example2
SUBDIRS += example3
SUBDIRS += GenerateGrid3D
SUBDIRS += GenerateRigid
SUBDIRS += SofaMT
#SUBDIRS += SofaKaapi
system(pkg-config --exists flowvr-render):SUBDIRS += SofaFlowVR
