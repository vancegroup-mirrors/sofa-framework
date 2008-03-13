#include <iostream>
#include <fstream>

#include "Sofa-old/Components/Graph/Simulation.h"
#include "Sofa-old/Components/Graph/Action.h"
#include "Sofa-old/Components/Common/Factory.h"
#include "Sofa-old/Components/Thread/CTime.h"
#include "Sofa-old/Components/Thread/Automate.h"
#include "Sofa-old/Components/Thread/ThreadSimulation.h"
#include "Sofa-old/Components/Thread/ExecBus.h"
#include "Sofa-old/Components/Thread/Node.h"

//#define VERBOSE

using Sofa::Components::Thread::CTime;
using Sofa::Components::Thread::ctime_t;

using namespace Sofa::Components::Graph;

namespace Sofa { namespace Components { namespace Thread {
extern Graph::GNode* groot;
} } }

namespace Projects
{

namespace SofaMT
{

using namespace Sofa::Components::Thread;

int nbIter = 0;
volatile int curIter = 0;

class DrawCB : public Automate::DrawCB
{
    void drawFromAutomate()
    {
        int n1 = curIter*80/(nbIter-1);
        ++curIter;
        int n2 = curIter*80/(nbIter-1);
        while(n2>n1)
        {
            std::cout << '.' << std::flush;
            ++n1;
        }
        //sleep(1);
    }
};
DrawCB drawCB;

void init(GNode* groot, int nthread)
{
    ThreadSimulation::getInstance()->initInstance("simpleMonoFrequency",nthread-1);
    groot->setMultiThreadSimulation(true);
    Sofa::Components::Thread::groot = groot;
    Automate::setDrawCB(&drawCB);
}

void run(int niter)
{
    ThreadSimulation::getInstance()->start();
    nbIter = niter;
    curIter = 0;
    Node* n = NULL;
    ExecBus *xbus = ExecBus::getInstance();
    while (curIter < nbIter)
    {
        n = xbus->getNext("displayThread", n);
        if (n)
        {
            n->execute("displayThread");
        }
    }
}

} // namespace SofaMT

} // namespace Projects

// ---------------------------------------------------------------------
// --- MAIN
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: "<<argv[0]<<" filename.scn nthreads [niterations]\n";
        return 1;
    }
    std::string fileName = argv[1];
    std::string nthreadStr = argv[2];
    int nthread = atoi(nthreadStr.c_str());
    int nbIter = (argc>=4)?atoi(argv[3]):500;
    
    GNode* groot = NULL; 
    ctime_t t0, t1;
    CTime::getRefTime();
    
    if (!fileName.empty())
    {
        groot = Simulation::load(fileName.c_str());
    }
    
    if (groot==NULL)
    {
        groot = new GNode;
    }
    groot->setAnimate(true);
    
    if (nthread != 0)
    {
        std::cout << "Initializing " << nthread << " threads." << std::endl;
        Projects::SofaMT::init(groot, nthread);
    }
    
    std::cout << "Computing first iteration." << std::endl;
    
    Simulation::animate(groot);
    
    //=======================================
    // Run the main loop
    
    std::cout << "Computing " << nbIter << " iterations on " << nthread << " thread(s)." << std::endl;
    t0 = CTime::getRefTime();
    
    if (nthread == 0)
    {
        //=======================================
        // SEQUENTIAL MODE
        int n = 0;
        for (int i=0;i<nbIter;i++)
        {
            int n2 = i*80/(nbIter-1);
            while(n2>n)
            {
                std::cout << '.' << std::flush;
                ++n;
            }
            Simulation::animate(groot);
        }
    }
    else
    {
        //=======================================
        // PARALLEL MODE
        Projects::SofaMT::run(nbIter);
    }
    
    t1 = CTime::getRefTime();
    std::cout << std::endl;
    std::cout << nbIter << " iterations done." << std::endl;
    std::cout << "Time: " << ((t1-t0)/(CTime::getRefTicksPerSec()/1000))*0.001 << " seconds, " << ((t1-t0)/(CTime::getRefTicksPerSec()/1000))/(double)nbIter <<" ms/it." << std::endl;
    std::string logname = fileName.substr(0,fileName.length()-4)+"-log-"+nthreadStr+".txt";
    std::ofstream flog(logname.c_str());
    flog << "Time: " << ((t1-t0)/(CTime::getRefTicksPerSec()/1000))*0.001 << " seconds, " << ((t1-t0)/(CTime::getRefTicksPerSec()/1000))/(double)nbIter <<" ms/it." << std::endl;
    flog.close();
    std::string objname = fileName.substr(0,fileName.length()-4)+"-scene-"+nthreadStr+".obj";
    std::cout << "Exporting to OBJ " << objname << std::endl;
    Simulation::exportOBJ(groot, objname.c_str());
    
    return 0;
}
