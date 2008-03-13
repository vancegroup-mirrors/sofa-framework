#include <iostream>
#include "argumentParser.h"
#include "Sofa-old/Components/Graph/Simulation.h"
#ifdef SOFA_GUI_FLTK
#include "Sofa-old/GUI/FLTK/Main.h"
#endif
#ifdef SOFA_GUI_QT
#include "Sofa-old/GUI/QT/Main.h"
#endif

using std::cout;
using std::cerr;
using std::endl;
#include <string>
using std::string;

#include "Sofa-old/Components/Graph/GNode.h"
#include "Sofa-old/Components/XML/XML.h"
#include "Sofa-old/Components/Graph/PrintAction.h"
#include "Sofa-old/Components/Graph/AnimateAction.h"

//----------------------------------------
// Stuff used in procedural scene building

#include "Sofa-old/Components/Gravity.h"
#include "Sofa-old/Components/CoordinateSystem.h"
#include "Sofa-old/Components/EulerSolver.h"
#include "Sofa-old/Components/RungeKutta4Solver.h"
#include "Sofa-old/Components/CGImplicitSolver.h"
#include "Sofa-old/Components/StaticSolver.h"
#include "Sofa-old/Components/UniformMass.h"
#include "Sofa-old/Components/FixedConstraint.h"
#include "Sofa-old/Core/MechanicalObject.h"
#include "Sofa-old/Components/RegularGridTopology.h"
#include "Sofa-old/Core/Context.h"
#include "Sofa-old/Contrib/Testing/FEMcontact/TetrahedralFEMForceField.inl"
#include "Sofa-old/Contrib/Testing/FEMcontact/PlaneForceField.inl"
#include <GL/glut.h>

using namespace Sofa::Components;
using namespace Sofa::Components::Graph;
using namespace Sofa::Core;
using namespace Sofa::Abstract;

namespace Projects
{

namespace example3
{ // just to group custom methods and variables

typedef Sofa::Components::Common::Vec3Types DOFs;
typedef DOFs::Deriv Vec3;

// Stuff used in procedural scene building
//----------------------------------------

GNode* FEMCube(Vec3 center, double size, int method)
{
        GNode* node= new GNode("cube");

        // create a mechanical object
        MechanicalObject<DOFs>* cube = new MechanicalObject<DOFs>;
        node->addObject(cube);
        cube->setName("FEM cube");

        // create a regular grid
        RegularGridTopology *cubeShape = new RegularGridTopology(3,3,3);
        cubeShape->setPos(center[0] - size/2.0, center[0] + size/2.0,
                          center[1] - size/2.0, center[1] + size/2.0,
                          center[2] - size/2.0, center[2] + size/2.0);
        node->addObject(cubeShape);

        // create a FEM force field and link the grid to the force field
        TetrahedralFEMForceField<DOFs>* forceField = new TetrahedralFEMForceField<DOFs>(cube);
        node->addObject(forceField);
        forceField->setMethod(method);
        forceField->setYoungModulus(1000);

        UniformMass<DOFs,double>* mass = new UniformMass<DOFs,double>(cube);
        node->addObject(mass);
        mass->setMass( 1 );

        // Create plane reaction force field
        PlaneForceField<DOFs> *planeReaction = new PlaneForceField<DOFs>(cube, forceField);
        node->addObject(planeReaction);
        planeReaction->setPlane(Vec3(0,1,0), -10);

        return node;
}

GNode* buildMyObjects(void)
{
        GNode* groot = new GNode;
        groot->setName("root");

        GNode* cube1 = FEMCube(Vec3(-15,0,0), 10.0, TetrahedralFEMForceField<DOFs>::SMALL);
        groot->addChild( cube1 );

        GNode* cube2 = FEMCube(Vec3(0,0,0), 10.0, TetrahedralFEMForceField<DOFs>::LARGE);
        groot->addChild( cube2 );

        GNode* cube3 = FEMCube(Vec3(15,0,0), 10.0, TetrahedralFEMForceField<DOFs>::POLAR);
        groot->addChild( cube3 );

        CGImplicitSolver* solver = new CGImplicitSolver;
        solver->f_maxIter.setValue(200);
        solver->f_printLog.setValue(false);
        groot->addObject( solver );
        
        Gravity* gravity = new Gravity;
        gravity->f_gravity.setValue(Vec3(0,-1,0) ) ; 
        groot->addObject( gravity );
        
        groot->getContext()->setDt(0.01f);
        
        Simulation::init(groot);

        return groot;
}

} // namespace example3
} // namespace Projects
using namespace Projects::example3;

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

        //==============================================
        // begin build the scene

        GNode* groot = buildMyObjects();

        // end build the scene
        //===============================================

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

        //===========================================
        // Run the main loop

        if (startAnim)
                groot->getContext()->setAnimate(false);

        groot->getContext()->setAnimate(startAnim);

        if (gui=="none") {
                cout << "Computing 1000 iterations." << endl;
                for (int i=0;i<1000;i++) {
                        Sofa::Components::Graph::Simulation::animate(groot);
                }
                cout << "1000 iterations done." << endl;
        }
#ifdef SOFA_GUI_FLTK
        else if (gui=="fltk") {
                Sofa::GUI::FLTK::MainLoop(argv[0]);
        }
#endif
#ifdef SOFA_GUI_QT
        else if (gui=="qt") {
                Sofa::GUI::QT::MainLoop(argv[0],groot);
        }
#endif
        else {
                cerr << "Unsupported GUI."<<endl;
                exit(1);
        }
        return 0;
}



