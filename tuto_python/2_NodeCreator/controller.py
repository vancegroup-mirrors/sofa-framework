import Sofa
import tree

class Controller(Sofa.PythonScriptController):

	# called once the script is loaded
	def onLoaded(self,node):
		self.rootNode = node
		return 0

	# optionnally, script can create a graph...
	def createGraph(self,node):
		tree.createTree(node)
		return 0

	# key and mouse events; use this to add some user interaction to your scripts 
	def onKeyPressed(self,k):
		print 'onKeyPressed '+k
		if k=='A':
			child=self.rootNode.createChild('Child')
			tree.createTree(child)
		return 0 
 
 