
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include "argumentParser.h"

#include "Sofa-old/Components/Graph/Simulation.h"
#include "Sofa-old/Components/Graph/GNode.h"
#include "Sofa-old/Components/XML/XML.h"
#include "Sofa-old/Components/Graph/PrintAction.h"
#include "Sofa-old/Components/Graph/AnimateAction.h"

#ifdef SOFA_GUI_FLTK
#include "Sofa-old/GUI/FLTK/Main.h"
#endif
#ifdef SOFA_GUI_QT
#include "Sofa-old/GUI/QT/Main.h"
#endif

#include "ProceduralScene.h"
using namespace Projects::procedural;

#include <Sofa-old/Components/EulerSolver.h>
using namespace Sofa::Components;
using namespace Sofa::Components::Graph;

//----------------------------------------
// begin my new stuff
#include "EulerStabilizedSolver.h"
SOFA_LINK_CLASS(EulerStabilizedSolver)

// end my stuff
//----------------------------------------



int main(int argc, char** argv)
{
    // Default parameters
    std::string fileName="";
    bool buildSceneProcedurally=false;
    bool startAnim=false;
    std::string gui;
#ifdef SOFA_GUI_FLTK

    gui = "fltk";
#endif
#ifdef SOFA_GUI_QT

    gui = "qt";
#endif

    // Read the command line
    parse("This is a SOFA application. Here are the command line arguments")
    .option(&fileName,'f',"file","scene file")
    .option(&buildSceneProcedurally,'p',"proceduralScene","build scene procedurally instead of reading it from a file")
    .option(&startAnim,'s',"start","start the animation loop")
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


    //=======================================
    // begin create the scene
    GNode* groot = new GNode;

    if (buildSceneProcedurally || fileName=="")
    {
        groot->addObject( (new Gravity)->setGravity(Vec3(0,0,0)) );
        groot->addObject( (new EulerStabilizedSolver)->setPrintLog(false) );
        //groot->setDt(1);
        //groot->addObject( (new EulerSolver)->setPrintLog(true) );
        
        ParticleString* particles = new ParticleString(2);
        groot->addChild( particles->getRoot() );
        
        Simulation::init(groot);
    }
    else
    {
        groot = Simulation::load(fileName.c_str());
    }

    if (groot==NULL)
    {
        std::cerr << "Graph creation failed.\n";
        return 1;
    }

    // end create scene
    //=======================================
    // begin examples of actions, for illustration only

    cout<<"PrintAction: --------------------"<<endl;
    groot->execute<PrintAction>();
    cout<<"----------------- end PrintAction"<<endl;

    // Enumerate all visual models:
    std::vector<VisualModel*> vmodels;
    groot->getTreeObjects<VisualModel>(&vmodels);
    std::cout << vmodels.size() << " visual models:";
    for (unsigned int i=0;i<vmodels.size();++i)
        std::cout << " " << vmodels[i]->getName();
    std::cout << std::endl;

    // end examples for illustration
    //=======================================
    // Run the main loop

    groot->getContext()->setAnimate(startAnim);

    if (gui=="none")
    {
        cout << "Computing 1000 iterations." << endl;
        for (int i=0;i<1000;i++)
        {
            Sofa::Components::Graph::Simulation::animate(groot);
        }
        cout << "1000 iterations done." << endl;
    }
#ifdef SOFA_GUI_FLTK
    else if (gui=="fltk")
    {
        Sofa::GUI::FLTK::MainLoop(argv[0]);
    }
#endif
#ifdef SOFA_GUI_QT
    else if (gui=="qt")
    {
        Sofa::GUI::QT::MainLoop(argv[0],groot);
    }
#endif
    else
    {
        cerr << "Unsupported GUI."<<endl;
        exit(1);
    }
    return 0;
}


