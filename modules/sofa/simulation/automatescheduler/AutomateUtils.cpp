/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <iostream>

//J. Allard: remove dependencies to GUI
//#include "Sofa/GUI/FLTK/GUI.h"
#include <sofa/core/BehaviorModel.h>
#include <sofa/helper/system/config.h>
#include <sofa/simulation/tree/Simulation.h>
#include <sofa/simulation/tree/AnimateVisitor.h>
#include <sofa/simulation/tree/CollisionVisitor.h>
#include <sofa/simulation/tree/UpdateMappingVisitor.h>
#include <sofa/simulation/tree/VisualVisitor.h>

#include <sofa/simulation/automatescheduler/AutomateUtils.h>
#include <sofa/simulation/automatescheduler/Automate.h>
#include <sofa/simulation/automatescheduler/ObjSubAutomate.h>
#include <sofa/simulation/automatescheduler/StateMachine.h>
#include <sofa/helper/system/thread/CTime.h>

namespace sofa
{

namespace simulation
{

namespace automatescheduler
{

void drawCode(void)
{
	if (Automate::getInstance()->drawCB != NULL)
	{
		Automate::getInstance()->execDrawCB();
	}
    else	
	{
		emptyDrawCode();
	}
}

MonoFrequencyObjSubAutomate* subAutomate;
simulation::tree::GNode* groot = NULL;

class ObjectUpdater : public simulation::tree::AnimateVisitor
{
public:
	ObjectUpdater(double dt)
	{
		setDt(dt);
	}
	virtual void update() = 0;
};

class BehaviorModelUpdater : public ObjectUpdater
{
public:
	simulation::tree::GNode* node;
	core::BehaviorModel* bm;
	BehaviorModelUpdater(simulation::tree::GNode* n, core::BehaviorModel* m, double dt)
	: ObjectUpdater(dt), node(n), bm(m)
	{
	}

	virtual void update()
	{
		processBehaviorModel(node, bm);
	}
};

class SolverUpdater : public ObjectUpdater
{
public:
	simulation::tree::GNode* node;
	core::componentmodel::behavior::OdeSolver* solver;
	SolverUpdater(simulation::tree::GNode* n, core::componentmodel::behavior::OdeSolver* s, double dt)
	: ObjectUpdater(dt), node(n), solver(s)
	{
	}

	virtual void update()
	{
		processOdeSolver(node, solver);
	}
};

std::vector<ObjectUpdater*> objects;

ctime_t memTime;

void obj_nextStepCode(std::string bmName)
{
	//(Scene::getInstance()->searchBehaviorModel(bmName))->updatePosition(Scene::getInstance()->getDt());
	int i = atoi(bmName.c_str());
	if ((unsigned)i < objects.size())
	{
		objects[i]->update();
	}
}

void collisCode()
{
	// Scene::getInstance()->updateCollisionDetection();
	if (groot != NULL && groot->getAnimate())
	{
		groot->execute<simulation::tree::CollisionVisitor>();
	}
}

void collisResetCode()
{
	// Scene::getInstance()->updateCollisionReset();
	if (groot != NULL && groot->getAnimate())
	{
		groot->execute<simulation::tree::CollisionResetVisitor>();
	}
}

void collisDetectionCode()
{
	// Scene::getInstance()->updateCollisionDetection();
	if (groot != NULL && groot->getAnimate())
	{
		groot->execute<simulation::tree::CollisionDetectionVisitor>();
	}
}

void collisResponseCode()
{
	// Scene::getInstance()->updateCollisionResponse();
	if (groot != NULL && groot->getAnimate())
	{
		groot->execute<simulation::tree::CollisionResponseVisitor>();
	}
}

void emptyDrawCode()
{
    std::cerr << "ERROR: emptyDrawCode called in AutomateUtils.cpp\n";
}

void periphCode()
{	
	/// \todo update for scenegraph
	//(Scene::getInstance()->searchBehaviorModel("ManipMouse1"))->updatePosition(Scene::getInstance()->getDt());
}

//
class MyAnimateVisitor : public simulation::tree::AnimateVisitor
{
	virtual void processBehaviorModel(simulation::tree::GNode* node, core::BehaviorModel* obj)
	{
		objects.push_back(new BehaviorModelUpdater(node, obj, getDt()));
	}

	virtual void processSolver(simulation::tree::GNode* node, core::componentmodel::behavior::OdeSolver* obj)
	{
		objects.push_back(new SolverUpdater(node, obj, getDt()));
	}
};

void beginLoopFunc()
{
	unsigned int oldsize = objects.size();
	// First clear old updaters
	for (unsigned int i=0;i<objects.size();i++)
		delete objects[i];
	objects.clear();
	// Then execute a modified AnimateVisitor that store in objects the list of real computations
	if (groot!=NULL && groot->getAnimate())
		groot->execute<MyAnimateVisitor>();
	if (objects.size()!=oldsize)
		std::cout << oldsize << " -> " << objects.size() << " animated objects." << std::endl;
	// Finally regenerate the subautomate task list
	if (subAutomate!=NULL)
	{
		unsigned int actsize = subAutomate->getVisitorList().size();
		if (actsize < objects.size())
		{
			std::vector< MonoFrequencyObjSubAutomateAttribute* > tmp = subAutomate->getVisitorList();
			for (unsigned int i=tmp.size(); i< objects.size(); i++)
			{
				char buf[16];
				snprintf(buf,sizeof(buf),"%d",i);
				tmp.push_back(new MonoFrequencyObjSubAutomateAttribute(obj_nextStepCode, buf));
			}
			subAutomate->setVisitorList(tmp);
			subAutomate->setExitCondition(tmp.size());
			std::cout << "-- SCHEDULING -- "<<objects.size()<<" independant objects in the scene."<<std::endl;
		}
	}
}

void endLoopFunc()
{
	if (groot!=NULL && groot->getAnimate())
	{
		groot->execute<simulation::tree::UpdateMappingVisitor>();
		groot->execute<simulation::tree::VisualUpdateVisitor>();
	}
}

int compteurB;

void rezB()
{
  //	ctime_t testTime = MesureTemps();
  //	ctime_t timeTicks = TicksPerSec();

//	std::cout << ((testTime - memTime) / float(timeTicks)) * 1000 << std::endl;

	compteurB = 0;
}

void incrB()
{
	compteurB++;
}

bool testB()
{
	return compteurB == 2;
}

//

int nbIterations = 500000;

void decompteur()
{ 
	nbIterations--;
}

bool isFini()
{
	return nbIterations==0;
}

void exitCall()
{ 
	std::cout << "Simulation : done\n";
	StateMachine::getInstance()->onExit();
	exit(0); 
}

// Test

int MAX = 10;

void o0()
{
	/// \todo update for scenegraph
	/*
	if (Scene::getInstance()->getAnimate())
	{	
		(Scene::getInstance()->searchBehaviorModel("TitispringMass"))->updatePosition(Scene::getInstance()->getDt());
	}
	*/
}

void o1()
{
	/// \todo update for scenegraph
	/*
	if (Scene::getInstance()->getAnimate())
	{	
		(Scene::getInstance()->searchBehaviorModel("TataspringMass"))->updatePosition(Scene::getInstance()->getDt());
	}
	*/
}

void o2()
{
	/// \todo update for scenegraph
	/*
	if (Scene::getInstance()->getAnimate())
	{	
		(Scene::getInstance()->searchBehaviorModel("TotospringMass"))->updatePosition(Scene::getInstance()->getDt());
	}
	*/
}

void o3()
{
	/// \todo update for scenegraph
	/*
	if (Scene::getInstance()->getAnimate())
	{	
		(Scene::getInstance()->searchBehaviorModel("_TitispringMass"))->updatePosition(Scene::getInstance()->getDt());
	}
	*/
}

int compteurA;

void rezA()
{
	memTime = CTime::getTime();
	compteurA = 0;
}

void incrA()
{
	compteurA++;
}

bool testA()
{
	return compteurA == 4;
}

} // namespace automatescheduler

} // namespace simulation

} // namespace sofa
