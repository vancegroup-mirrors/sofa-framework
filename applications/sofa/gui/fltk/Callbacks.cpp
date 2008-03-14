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
// ---------------------------------------------------------------------
// --- Callbackd for the GUI
// --- Do NOT use GLUT callback only FLTK callbacks - Display & Reshape 
// --- callbacks are handled "automatically" by FLTK. Only define other 
// --- callbacks (Idle, mouse, keys, etc...)
// ---------------------------------------------------------------------

#include "GUI.h"
#include "Callbacks.h"
#include <sofa/simulation/tree/Simulation.h>
#include <sofa/simulation/tree/Simulation.h>
#include <sofa/simulation/automatescheduler/Automate.h>
#include <sofa/simulation/automatescheduler/ThreadSimulation.h>
#include <sofa/helper/system/thread/CTime.h>

namespace sofa { namespace simulation { namespace automatescheduler {
extern sofa::simulation::tree::GNode* groot;
} } }


namespace sofa
{

namespace gui
{

namespace fltk
{

using std::cout;
using std::endl;
using namespace sofa::simulation::automatescheduler;

extern UserInterface*	GUI;

// ---------------------------------------------------------------------
// --- 
// ---------------------------------------------------------------------
void LoadFileCB()
{
	char*	newFile;
	GUI->_currentFilename[0] = '\0';
	newFile = fl_file_chooser("Open File", "*.dat", GUI->_currentFilename,
							  false);
	if (newFile != NULL)
	{
		printf("Loading file '%s'\n", newFile);

		// GUI->viewer->LoadSpringMassModel(newFile);
	}
}


// ---------------------------------------------------------------------
// --- 
// ---------------------------------------------------------------------
void SaveFileCB(void)
{
	if (GUI->_currentFilename[0] == '\0')
	{
		//SaveFileAsCB();   	
	}
	else
	{
		//SaveFile(GUI->_currentFilename);
	}
}


// ---------------------------------------------------------------------
// --- 
// ---------------------------------------------------------------------
void SaveFileAsCB(void)
{
	char*	newFile;
	newFile = fl_file_chooser("Save File As...", "*.dat", NULL, true);
	if (newFile != NULL)
	{
		printf("Saving file '%s'\n", newFile);

		//SaveFile(newFile);
		strcpy(GUI->_currentFilename, newFile);
	}
}


// ---------------------------------------------------------------------
// --- 
// ---------------------------------------------------------------------
void QuitCB()
{
	int	answer	= fl_ask("Are you sure you want to quit?");
	if (answer == 1)
	{
		delete GUI->viewer;
		exit(0);
	}
}

// ---------------------------------------------------------------------
// --- 
// ---------------------------------------------------------------------
void IdleCB(void*)
{
	if (GUI != NULL)
	{
		if (GUI->viewer->GetScene()->getMultiThreadSimulation())
			return;
		// compute deformation, do the mapping, and update the OBJ model
		if (GUI->viewer->GetScene()->getAnimate())
		{
			sofa::simulation::tree::Simulation::animate(GUI->viewer->GetScene());
			//GUI->viewer->GetScene()->updateMappings();
			//CollisionEnvironment::getInstance()->startCollisionDetection(); //updateCollisionDetection();
			eventNewStep();
		}

		// call the rerender the graphics
		GUI->viewer->Animate();
	}
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
FLTKDrawCB FLTKDrawCB::instance;
void FLTKDrawCB::drawFromAutomate()
{
	if (GUI != NULL)
	{		
		GUI->viewer->redraw();
		eventNewStep();
	}
}


// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void ChangeForceIntensityCB(double /*value*/)
{
//	GUI->viewer->GetScene()->setLocalForcesScalingFactor(value / 100.0);
} 

// ---------------------------------------------------------
// ---
// ---------------------------------------------------------
void SmoothOnOffCB(bool value)
{
	GUI->viewer->GetScene()->setShowForceFields(value);
	Simulation::updateVisualContext(GUI->viewer->GetScene());
	if (value)
	{
		cout << "Smooth value set to TRUE" << endl;
	}
	else
	{
		cout << "Smooth value set to FALSE" << endl;
	}
}
void CollisionOnOffCB(bool value)
{
	GUI->viewer->GetScene()->setShowCollisionModels(value);
	Simulation::updateVisualContext(GUI->viewer->GetScene());
	if (value)
	{
		cout << "Collision value set to TRUE" << endl;
	}
	else
	{
		cout << "Collision value set to FALSE" << endl;
	}
}
void BehaviorOnOffCB(bool value)
{
	GUI->viewer->GetScene()->setShowBehaviorModels(value);
	Simulation::updateVisualContext(GUI->viewer->GetScene());
	if (value)
	{
		cout << "Behavior value set to TRUE" << endl;
	}
	else
	{
		cout << "Behavior value set to FALSE" << endl;
	}
}
void VisualOnOffCB(bool value)
{
	GUI->viewer->GetScene()->setShowVisualModels(value);
	Simulation::updateVisualContext(GUI->viewer->GetScene());
	if (value)
	{
		cout << "Visual value set to TRUE" << endl;
	}
	else
	{
		cout << "Visual value set to FALSE" << endl;
	}
}
void MappingOnOffCB(bool value)
{
	GUI->viewer->GetScene()->setShowMappings(value);
	Simulation::updateVisualContext(GUI->viewer->GetScene());
	if (value)
	{
		cout << "Mapping value set to TRUE" << endl;
	}
	else
	{
		cout << "Mapping value set to FALSE" << endl;
	}
}

void IdleAutomateCB(void*);

void AddThreadCB()
{
	char buf[50];

	if (!GUI->viewer->GetScene()->getMultiThreadSimulation())
	{
		GUI->viewer->GetScene()->setMultiThreadSimulation(true);
		groot = GUI->viewer->GetScene(); 
                Automate::setDrawCB(&FLTKDrawCB::instance);
		// Threads Management
		if (!ThreadSimulation::initialized())
			ThreadSimulation::getInstance()->initInstance("simpleMonoFrequency",1);
		ThreadSimulation::getInstance()->start();

		// start everything

		Fl::remove_idle(IdleCB);
		Fl::add_idle(IdleAutomateCB);
	}
	else
		ThreadSimulation::getInstance()->addThread();
	sprintf(buf, "%d Threads", ThreadSimulation::getInstance()->getNumCPU() + 1);
	GUI->threadLabel->value(buf);
}

void displayAutomateCB(void *)
{
	GUI->viewer->Animate();
}

void displayFPSCB(int FPS)
{
	char buf[50];
	sprintf(buf, "%d FPS", FPS);
	GUI->fpsLabel->value(buf);
}

void DtCB(double value)
{
	if (value!=0.0)
	{
		GUI->viewer->GetScene()->setDt(value);
		cout << "Dt value set to "<<value << endl;
	}
}

void    eventNewStep()
{
	static ctime_t beginTime = CTime::getTime();
	static const ctime_t timeTicks = CTime::getTicksPerSec();
	static int frameCounter = 0;

	if ((++frameCounter) == 10)
	{
		ctime_t curtime = CTime::getTime();
		double fps = ((double)timeTicks / (curtime - beginTime))*frameCounter;
		char buf[100];
		sprintf(buf, "%.1f FPS", fps);
		GUI->fpsLabel->value(buf);
		beginTime = curtime;
		frameCounter = 0;
	}
}


} // namespace fltk

} // namespace gui

} // namespace sofa
