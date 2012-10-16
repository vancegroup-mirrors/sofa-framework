import Sofa
import ObjectCreator
import random

# this "thing" creates and throws balls

class Controller(Sofa.PythonScriptController):
	# called once graph is created, to init some stuff...
	def createGraph(self,node):
	#	dragon = ObjectCreator.createDragon(node,'GreenDragon',0,10,0,'green')
	#	dragon = ObjectCreator.createDragon(node,'BlueDragon',0,30,0,'blue')
	#	dragon = ObjectCreator.createDragon(node,'RedDragon',0,50,0,'red')
		
	#	ObjectCreator.createArmadillo(node,'GreenArmadillo',0,10,0,'green')
	#	ObjectCreator.createArmadillo(node,'BlueArmadillo',0,30,0,'blue')
	#	ObjectCreator.createArmadillo(node,'RedArmadillo',0,50,0,'red')
		return 0

	def initGraph(self,node):
		print 'Gun.initGraph called (python side)'
		self.rootNode = node.getRoot()
		
	#	fixed = ObjectCreator.createObstacle(rootNode,'mesh/ball.obj','mesh/ball.obj','FFFFFFFF',[0.0, 0.0, 0.0],[0.0, 0.0, 0.0])

		#dragon = ObjectCreator.createDragon(rootNode)
		
		return 0
		
	 
	# key and mouse events; use this to add some user interaction to your scripts 
	def onKeyPressed(self,k):
		fireAngle = random.uniform(-50,-30)
		if k=='A':
			print 'Armadillo, go!'
			ObjectCreator.createArmadillo(self.rootNode,'RedArmadillo',0,50,0,'red')
		if k=='D':
			print 'Dragon launched!'
			obj = ObjectCreator.createDragon(self.rootNode,'GreenDragon',0,50,0,'green')
			#obj.findData('restVelocity').value=[0.0, 20.0, 0.0 ]
		# LEFT key : left
		if ord(k)==18:
			print 'Cube launched (left)!'
			obj = ObjectCreator.createCube(self.rootNode,'Cube',-50,50,0,60,fireAngle,0,'blue')
			#obj.findData('restVelocity').value=[0.0, 20.0, 0.0 ]
		# RIGHT key : right
		if ord(k)==20:
			print 'Cube launched (right)!'
			obj = ObjectCreator.createCube(self.rootNode,'Cube',50,50,0,-60,fireAngle,0,'magenta')
			#obj.findData('restVelocity').value=[0.0, 20.0, 0.0 ]
		# UP key : front
		if ord(k)==19:
			print 'Cube launched (front)!'
			obj = ObjectCreator.createCube(self.rootNode,'Cube',0,50,50,0,fireAngle,-60,'yellow')
			#obj.findData('restVelocity').value=[0.0, 20.0, 0.0 ]
		# DOWN key : rear
		if ord(k)==21:
			print 'Cube launched (back)!'
			obj = ObjectCreator.createCube(self.rootNode,'Cube',0,50,-50,0,fireAngle,60,'cyan')
			#obj.findData('restVelocity').value=[0.0, 20.0, 0.0 ]
		return 0 
	 
 