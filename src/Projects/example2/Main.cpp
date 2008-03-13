#include <iostream>
#include "argumentParser.h"
#include "Sofa-old/Components/Graph/Simulation.h"
#ifdef SOFA_GUI_FLTK
#include "Sofa-old/GUI/FLTK/Main.h"
#endif
#ifdef SOFA_GUI_QT
#include "Sofa-old/GUI/QT/Main.h"
#endif
#include <GL/glut.h>

using std::cout;
using std::cerr;
using std::endl;
#include <string>
using std::string;

#include "Sofa-old/Components/Graph/GNode.h"
#include "Sofa-old/Components/XML/XML.h"
#include "Sofa-old/Components/Graph/PrintAction.h"
#include "Sofa-old/Components/Graph/AnimateAction.h"
#include "Sofa-old/Components/Gravity.h"
#include "Sofa-old/Components/EulerSolver.h"
#include "Sofa-old/Components/CGImplicitSolver.h"

#include "ProceduralScene.h"
using namespace Projects::procedural; 

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    glutInit(&argc,argv);
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
    

    GNode* groot = NULL;

/*    if (buildSceneProcedurally || fileName=="")
    {
        //=====================================================================
        // begin build the scene

        groot = new GNode;
        groot->setName("root"); 
    
        CGImplicitSolver* solver = new CGImplicitSolver;
        solver->f_printLog.setValue(false);
        solver->f_tolerance.setValue(1.0e-3);
        solver->f_smallDenominatorThreshold.setValue(1.0e-20);
        solver->f_maxIter.setValue(10);  
        groot->addObject( solver );
        
        Gravity* gravity = new Gravity;
        gravity->f_gravity.setValue(Vec3(0, 0,10));
        groot->addObject( gravity );
    
        int nbx = 5, nby=12;
        groot->addChild( procedural::buildRectangularMesh(nbx, nby,1.*nbx,1.*nby,1000,0.0,1) );   



    //=======================================
    // do not forget to initialize the scene
        groot->setAnimate(true);
        Simulation::init(groot);
        //Simulation::printXML(groot,"toto.scn"); 
    }*/
    if (buildSceneProcedurally || fileName=="")
    {
        //=====================================================================
        // begin build the scene

        groot = new GNode;
        groot->setName("root"); 
    
        ArticulatedSolid* solids = new ArticulatedSolid(2,1,Vec3(0,0,-1));
        groot->addChild( solids->getRoot() );   



    //=======================================
    // do not forget to initialize the scene
        groot->setAnimate(true);
        Simulation::init(groot);
        //Simulation::printXML(groot,"toto.scn"); 
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

    // end set the global parameters
    //=======================================
    // begin examples for illustration

//     cout<<"PrintAction: --------------------"<<endl;
//     groot->execute<PrintAction>();
//     cout<<"----------------- end PrintAction"<<endl;
// 
//     // Enumerate all visual models:
//     std::vector<VisualModel*> vmodels;
//     groot->getTreeObjects<VisualModel>(&vmodels);
//     std::cout << vmodels.size() << " visual models:";
//     for (unsigned int i=0;i<vmodels.size();++i)
//         std::cout << " " << vmodels[i]->getName();
//     std::cout << std::endl;

    // end examples for illustration
    //=======================================
    // Run the main loop
    std::cout << "Run the main loop";

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

