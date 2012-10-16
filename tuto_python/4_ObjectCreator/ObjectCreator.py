import Sofa

# utility python module to dynamically create objects in Sofa


def createDragon(parentNode,name,x,y,z,color):
	node = parentNode.createChild(name)
#        <EulerImplicit name="cg_odesolver" printLog="false" />
	node.createObject(Sofa.BaseObjectDescription('cg_odesolver','EulerImplicit'))
#        <CGLinearSolver iterations="25" name="linear solver" tolerance="1.0e-9" threshold="1.0e-9" />
	desc = Sofa.BaseObjectDescription('linear solver','CGLinearSolver')
	desc.setAttribute('iterations','25')
	desc.setAttribute('tolerance','1.0e-9')
	desc.setAttribute('threshold','1.0e-9')
	node.createObject(desc)

#        <MechanicalObject dx="20" dy="20" dz="29" rx="33" />
	desc = Sofa.BaseObjectDescription('mObject','MechanicalObject')
	object = node.createObject(desc)

#        <UniformMass totalmass="10" />
	mass = node.createObject(Sofa.BaseObjectDescription('mass','UniformMass'))
	mass.findData('totalmass').value=10

#        <RegularGrid nx="6" ny="5" nz="3" xmin="-11" xmax="11" ymin="-7" ymax="7" zmin="-4" zmax="4" />
	desc = Sofa.BaseObjectDescription('RegularGrid1','RegularGrid')
	desc.setAttribute('nx','6')
	desc.setAttribute('ny','5')
	desc.setAttribute('nz','3')
	desc.setAttribute('xmin','-11')
	desc.setAttribute('xmax','11')
	desc.setAttribute('ymin','-7')
	desc.setAttribute('ymax','7')
	desc.setAttribute('zmin','-4')
	desc.setAttribute('zmax','4')
	node.createObject(desc)

#        <RegularGridSpringForceField name="Springs" stiffness="350" damping="1" />
	desc = Sofa.BaseObjectDescription('Springs','RegularGridSpringForceField')
	desc.setAttribute('stiffness','350')
	desc.setAttribute('damping','1')
	node.createObject(desc)

#        <Node name="VisuDragon" tags="Visual">
	VisuNode = node.createChild('VisuDragon')

#            <OglModel name="Visual" filename="mesh/dragon.obj" color="red" />
	desc = Sofa.BaseObjectDescription('Visual','OglModel')
	desc.setAttribute('filename','mesh/dragon.obj')
	desc.setAttribute('color',color)
	VisuNode.createObject(desc)

#            <BarycentricMapping object1="../.." object2="Visual" />
	desc = Sofa.BaseObjectDescription('BarycentricMapping','BarycentricMapping')
	desc.setAttribute('object1','@..')
	desc.setAttribute('object2','@Visual')
	VisuNode.createObject(desc)

#        <Node name="Surf">
	SurfNode = node.createChild('Surf')
#            <MeshObjLoader name="loader" filename="mesh/dragon.obj" />
	desc = Sofa.BaseObjectDescription('loader','MeshObjLoader')
	desc.setAttribute('filename','mesh/dragon.obj')
	SurfNode.createObject(desc)
#            <Mesh src="@loader" />
	desc = Sofa.BaseObjectDescription('mesh','Mesh')
	desc.setAttribute('src','@loader')
	SurfNode.createObject(desc)
#            <MechanicalObject src="@loader" />
	desc = Sofa.BaseObjectDescription('mecaObj','MechanicalObject')
	desc.setAttribute('src','@loader')
	SurfNode.createObject(desc)
#            <Triangle />
	SurfNode.createObject(Sofa.BaseObjectDescription('triangle','Triangle'))
#            <Line />
	SurfNode.createObject(Sofa.BaseObjectDescription('line','Line'))
#            <Point />
	SurfNode.createObject(Sofa.BaseObjectDescription('point','Point'))
#            <BarycentricMapping />
	SurfNode.createObject(Sofa.BaseObjectDescription('mapping','BarycentricMapping'))

	object.applyTranslation(x,y,z)

	return node






def createArmadillo(parentNode,name,x,y,z,color):
	node = parentNode.createChild(name)
#        <EulerImplicit name="cg_odesolver" printLog="false" />
	node.createObject(Sofa.BaseObjectDescription('cg_odesolver','EulerImplicit'))
#        <CGLinearSolver iterations="25" name="linear solver" tolerance="1.0e-9" threshold="1.0e-9" />
	desc = Sofa.BaseObjectDescription('linear solver','CGLinearSolver')
	desc.setAttribute('iterations','25')
	desc.setAttribute('tolerance','1.0e-9')
	desc.setAttribute('threshold','1.0e-9')
	node.createObject(desc)

#        <MechanicalObject dx="20" dy="20" dz="29" rx="33" />
	desc = Sofa.BaseObjectDescription('mObject','MechanicalObject')
	object = node.createObject(desc)

#        <UniformMass totalmass="10" />
	mass = node.createObject(Sofa.BaseObjectDescription('mass','UniformMass'))
	mass.findData('totalmass').value=10

#        <SparseGridTopology n="4 4 4" fileTopology="mesh/Armadillo_verysimplified.obj" />
	desc = Sofa.BaseObjectDescription('grid','SparseGridTopology')
	desc.setAttribute('fileTopology','mesh/Armadillo_verysimplified.obj')
	desc.setAttribute('n','4 4 4')
	node.createObject(desc)
	
#        <HexahedronFEMForceField youngModulus="100" />
	desc = Sofa.BaseObjectDescription('fem','HexahedronFEMForceField')
	desc.setAttribute('youngModulus','100')
	node.createObject(desc)
	
#        <Node name="VisuDragon" tags="Visual">
	VisuNode = node.createChild('Visu')

#            <OglModel name="Visual" filename="mesh/dragon.obj" color="red" />
	desc = Sofa.BaseObjectDescription('Visual','OglModel')
	desc.setAttribute('filename','mesh/Armadillo_verysimplified.obj')
	desc.setAttribute('color',color)
	VisuNode.createObject(desc)

#            <BarycentricMapping object1="../.." object2="Visual" />
	desc = Sofa.BaseObjectDescription('BarycentricMapping','BarycentricMapping')
	desc.setAttribute('object1','@..')
	desc.setAttribute('object2','@Visual')
	VisuNode.createObject(desc)

#        <Node name="Surf">
	SurfNode = node.createChild('Surf')
#            <MeshObjLoader name="loader" filename="mesh/dragon.obj" />
	desc = Sofa.BaseObjectDescription('loader','MeshObjLoader')
	desc.setAttribute('filename','mesh/Armadillo_verysimplified.obj')
	SurfNode.createObject(desc)
#            <Mesh src="@loader" />
	desc = Sofa.BaseObjectDescription('mesh','Mesh')
	desc.setAttribute('src','@loader')
	SurfNode.createObject(desc)
#            <MechanicalObject src="@loader" />
	desc = Sofa.BaseObjectDescription('mecaObj','MechanicalObject')
	desc.setAttribute('src','@loader')
	SurfNode.createObject(desc)
#            <Triangle />
	SurfNode.createObject(Sofa.BaseObjectDescription('triangle','Triangle'))
#            <Line />
	SurfNode.createObject(Sofa.BaseObjectDescription('line','Line'))
#            <Point />
	SurfNode.createObject(Sofa.BaseObjectDescription('point','Point'))
#            <BarycentricMapping />
	SurfNode.createObject(Sofa.BaseObjectDescription('mapping','BarycentricMapping'))


	object.applyTranslation(x,y,z)

	return node



