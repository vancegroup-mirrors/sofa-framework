#include <iostream>
#include "argumentParser.h"
using std::cout;
using std::cerr;
using std::endl;
#include <string>
using std::string;
#include <GL/glut.h>

#include "Sofa-old/Components/Graph/Simulation.h"
#include "Sofa-old/GUI/QT/Coin/SofaCoinManager.h"
using namespace Sofa::Components::Graph;
using namespace Sofa::GUI::QT;


int main(int argc, char** argv)
{
    /// Parameters 
    std::string fileName="";
    bool startAnim=false;

    // command line 
    parse("This is a SOFA application. Here are the command line arguments")
    .option(&fileName,'f',"file","scene file")
    .option(&startAnim,'s',"start","start the animation loop")
    (argc,argv);


    /// -- Build the scene 
    Sofa::Components::Graph::GNode* groot = NULL;
    groot = Simulation::load(fileName.c_str());
    if (groot==NULL)
    {
        std::cerr << "Graph creation failed.\n";
        return 1;
    }
    groot->getContext()->setAnimate(startAnim);
    
    /// -- Initialize 
    glutInit(&argc,argv);
    /// Initialize Qt, SoQt and return a widget for the SoQt viewer
    QWidget * soqtWindow = SoQt::init(argc, argv, argv[0]);
    if (soqtWindow == NULL)
        exit(1);
    /// initialize the custom inventor classes
    SofaCoinManager::initClasses();
    
    
    /// -- Organize the GUI.
    /// Sofa gui is a QMainWindow. It is not aware of soqtWindow.
    RealGUI* sofaGui = new RealGUI(groot, 0);

    /// Manage a SoQt viewer connected to sofa and using a viewer inserted in soqtWindow.
    /// We can not include soqtWindow in sofaGui (?) because SoQt::init returns a top-level widget.
    SofaCoinManager* myViewer = new SofaCoinManager(soqtWindow,sofaGui,groot);
    myViewer->buildMeshTopologies(false);
    myViewer->buildVisualModels(true);

    /// window geometry
    QPoint o(100,10); // origin of the Sofa gui
    QSize s(700,600); // size
    sofaGui->move( o );
    sofaGui->resize(s );
    soqtWindow->move( QPoint( o.x()+ s.width() + 20, o.y() ) ); // beside the Sofa gui 
    soqtWindow->resize(s);

    /// Show 
    myViewer->show(soqtWindow); 
    sofaGui->show();
    SoQt::show(soqtWindow);

     /// -- Start the main loop
    SoQt::mainLoop();


    return 0;
}


