#include <iostream>
#include <fstream>
#include "argumentParser.h"
#include "Sofa-old/Components/Graph/Simulation.h"
#include "Sofa-old/Components/Common/Factory.h"
#include "SensAble.h"
#ifdef SOFA_GUI_FLTK
#include "Sofa-old/GUI/FLTK/Main.h"
#endif
#ifdef SOFA_GUI_QT
#include "Sofa-old/GUI/QT/Main.h"
#endif

using namespace Sofa::Components::Graph;

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
	std::string fileName = "C:/Documents and Settings/SOFA/Desktop/MMVR/EyeCut3/EyeTopologyChange2Haptic.scn";
	bool        startAnim = false;
	bool        printFactory = false;
	std::string gui = "none";
#ifdef SOFA_GUI_FLTK
	gui = "fltk";
#endif
#ifdef SOFA_GUI_QT
	gui = "qt";
#endif

	parse("This is a SOFA application. Here are the command line arguments")
	.option(&fileName,'f',"file","scene file")
	.option(&startAnim,'s',"start","start the animation loop")
	.option(&printFactory,'p',"factory","print factory logs")
	.option(&gui,'g',"gui","choose the UI (none"
#ifdef SOFA_GUI_FLTK
		"|fltk"
#endif
#ifdef SOFA_GUI_QT
		"|qt"
#endif
		")"
	)
	(argc,argv);

	if (printFactory)
	{
		std::cout << "////////// FACTORY //////////" << std::endl;
		Sofa::Components::Common::printFactoryLog();
		std::cout << "//////// END FACTORY ////////" << std::endl;
	}

	Sofa::Components::Graph::GNode* groot = NULL; 

	groot = Sofa::Components::Graph::Simulation::load(fileName.c_str());

	if (groot==NULL)
		return 1;

	if (startAnim)
		groot->setAnimate(true);

	//=======================================
	// Add the haptic device
	SensAble* phantom = new SensAble;
	phantom->setName("phantom");
	GNode* node = groot->getChild("haptic");
	if (node == NULL)
	{
		node = new GNode;
		node->setName("haptic");
		groot->addChild(node);
	}
	node->addObject(phantom);
	//if (node->getMechanicalModel() == NULL)
	//	node->addObject(phantom->sphereModel);
	//GNode* cnode = new GNode;
	//cnode->setName("Collision");
	//node->addChild(cnode);
	//cnode->addObject(phantom->sphereModel);

	phantom->init();
	//phantom->sphereModel->init();


	//=======================================
	// Run the main loop

	if (gui=="none")
	{
		std::cout << "Computing 1000 iterations." << std::endl;
		for (int i=0;i<1000;i++)
		{
			Sofa::Components::Graph::Simulation::animate(groot);
		}
		std::cout << "1000 iterations done." << std::endl;
	}
#ifdef SOFA_GUI_FLTK
	else if (gui=="fltk") {
		Sofa::GUI::FLTK::MainLoop(argv[0],groot);
	}
#endif
#ifdef SOFA_GUI_QT
    else if (gui=="qt") {
		Sofa::GUI::QT::MainLoop(argv[0],groot, fileName.c_str(), true);
    }
#endif
	else {
		std::cerr << "Unsupported GUI."<<std::endl;
		exit(1);
	}
	return 0;
}
