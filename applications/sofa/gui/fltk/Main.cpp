/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the Free   *
* Software Foundation; either version 2 of the License, or (at your option)    *
* any later version.                                                           *
*                                                                              *
* This program is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for     *
* more details.                                                                *
*                                                                              *
* You should have received a copy of the GNU General Public License along with *
* this program; if not, write to the Free Software Foundation, Inc., 51        *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                    *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <iostream>

#include "GUI.h"
#include "Main.h"
#include "Callbacks.h"

#include <sofa/simulation/automatescheduler/ExecBus.h>
#include <sofa/simulation/automatescheduler/CPU.h>
#include <sofa/simulation/automatescheduler/Node.h>
#include <sofa/simulation/tree/xml/Element.h>
#include <sofa/simulation/automatescheduler/Automate.h>
#include <sofa/simulation/automatescheduler/ThreadSimulation.h>
#include <sofa/simulation/tree/Simulation.h>

#include <sofa/helper/system/thread/CTime.h>

namespace sofa { namespace simulation { namespace automatescheduler {
extern simulation::tree::GNode* groot;
} } }

namespace sofa
{

namespace gui
{

namespace fltk
{

bool			verboseMode		= false;	//false;
UserInterface*	GUI				= NULL;


using namespace sofa::simulation::automatescheduler;

/*
template<>
FnDispatcher<CollisionElement, bool>* FnDispatcher<CollisionElement, bool>::uniqInstance = NULL;
template<>
FnDispatcher<CollisionElement, DetectionOutput*>* FnDispatcher<CollisionElement, DetectionOutput*>::uniqInstance = NULL;
*/
ctime_t beginTime;

void IdleAutomateCB(void*)
{
    static Node* n = NULL;

    if( ExecBus::getInstance() != NULL )
    {
        //ctime_t currentTime = CTime::getTime();
        //ctime_t timeTicks = CTime::getTicksPerSec();

        //if (int((float)timeTicks / (currentTime - beginTime)) < 60)
            if (1)// ((currentTime - beginTime) > 4000000)
        {
            n = ExecBus::getInstance()->getNext("displayThread", n);

            if (n)
            {
                //ctime_t begin = CTime::getTime();
                n->execute("displayThread");
                //ctime_t end = CTime::getTime();
                //ctime_t timeTicks = CTime::getTicksPerSec();
                //	std::cout << "display" << " " << n->str << std::endl;
                //	std::cout << ((end - begin) / float(timeTicks)) * 1000 << std::endl;
				//if (!strcmp(n->str, "disp"))
				//	eventNewStep();
            }

            //beginTime = CTime::getTime();
        }
        else
        {
            n = ExecBus::getInstance()->getNext("fltk", n);

            if( n )
            {
                n->execute("fltk");
            }
        }
    }
}

const char* progname="";

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int MainLoop(const char* pname, GNode* groot)
{
	progname = pname;

    // create interface
    GUI = new UserInterface(groot);

    // show the GUI
    GUI->show();

    // Threads Management
    if (groot->getMultiThreadSimulation())
    {
      sofa::simulation::automatescheduler::groot = groot;
        // Threads Management
		Automate::setDrawCB(&FLTKDrawCB::instance);
		if (!ThreadSimulation::initialized())
            ThreadSimulation::getInstance()->initInstance("simpleMonoFrequency",1);
        ThreadSimulation::getInstance()->start();

        // start everything

        beginTime = CTime::getTime();

        Fl::add_idle(IdleAutomateCB);

        return Fl::run();

        //return 0;
    }
    else
    {
        Fl::add_idle(IdleCB);

        return Fl::run();
    }
}

} // namespace fltk

} // namespace gui

} // namespace sofa
