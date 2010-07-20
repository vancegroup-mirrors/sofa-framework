/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include "RealGUI.h"
#include "Main.h"

#include <sofa/simulation/common/Simulation.h>
#include <sofa/simulation/common/Node.h>
#include <iostream>
#include <qapplication.h>
#include <qpushbutton.h>
#include <qcheckbox.h>
#include <qlineedit.h>
#include <qtabwidget.h>
#ifndef SOFA_QT4
#include <qlistview.h>
#include <qheader.h>
#endif
 

namespace sofa
{
 
  namespace gui
  {

    namespace qt
    {

      RealGUI* gui = NULL;
      QApplication* application = NULL;

      const char* progname="";

      // ---------------------------------------------------------------------
      // ---
      // ---------------------------------------------------------------------

      int MainLoop(const char* pname, sofa::simulation::Node* groot, const char* filename)
      {
	progname = pname;
	{
	  int *argc= new int;
	  char* argv[1];
          *argc = 1;
	  argv[0] = strdup(progname);
	  application = new QApplication(*argc,argv);
	  free(argv[0]);
	}
	// create interface
	gui = new RealGUI("qt");
	if (groot)
          gui->setScene(groot, filename);
	//((RealGUI*) gui)->setTitle(filename);
	//gui->viewer->SwitchToPresetView();
	
	application->setMainWidget( gui );

	// show the gui
	gui->show();


	return application->exec();
      }

      bool InsertTab(QWidget* tab, const char* name)
      {
	if (gui==NULL) return false;
	gui->tabs->insertTab(tab,name);
	gui->tabs->showPage(tab);
	return true;
      }


	 
      //**********************************************************************************
      //TODO!!!!!!!!!!!
      void Redraw()
      {
		  
	if (gui==NULL) return;
	gui->viewer->getQWidget()->update();
      }

      sofa::simulation::Node* CurrentSimulation()   
      {
			  
	if (gui==NULL) return NULL;


	return gui->viewer->getScene();
      }


    } // namespace qt

  } // namespace gui

} // namespace sofa
