import Sofa

# Python version of the "oneParticleSample" in cpp located in applications/tutorials/oneParticle
def oneParticleSample(node):
 node.findData('name').value='oneParticleSample'
 node.findData('gravity').value=[0.0, -9.81, 0.0]
 solver = node.createObject(Sofa.BaseObjectDescription('solver','EulerSolver'))
 solver.findData('printLog').value = 'false'
 node.addObject(solver);
 particule_node = node.createChild('particle_node')
 particle = particule_node.createObject(Sofa.BaseObjectDescription('particle','MechanicalObject'))
 particle.resize(1)
 mass = particule_node.createObject(Sofa.BaseObjectDescription('mass','UniformMass'))
 mass.findData('mass').value=1.0
 return 0

