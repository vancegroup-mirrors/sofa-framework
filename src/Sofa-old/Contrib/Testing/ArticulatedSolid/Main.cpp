#include "Sofa-old/Components/Graph/Simulation.h"
#ifdef SOFA_GUI_FLTK
#include "Sofa-old/GUI/FLTK/Main.h"
using namespace Sofa::GUI::FLTK;
#endif
#ifdef SOFA_GUI_QT
#include "Sofa-old/GUI/QT/Main.h"
using namespace Sofa::GUI::QT;
#endif

using namespace Sofa::Components::Graph;

SOFA_LINK_CLASS(ArticulatedSolidNode)

int main(int argc, char** argv) 
{
  GNode* scene = Simulation::load((argc>=2)?argv[1]:"Data/test3.scn");
  MainLoop(argv[0], scene);
  return 0;
}
