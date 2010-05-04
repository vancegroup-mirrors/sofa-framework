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
#include <sofa/gui/qt/RealGUI.h>

#ifdef SOFA_GUI_QTOGREVIEWER 
#include "viewer/qtogre/QtOgreViewer.h"
#endif

#ifdef SOFA_GUI_QTVIEWER
#include "viewer/qt/QtViewer.h"
#endif

#ifdef SOFA_GUI_QGLVIEWER
#include "viewer/qgl/QtGLViewer.h"
#endif


#include <sofa/simulation/common/Simulation.h>

#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/WriteStateVisitor.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>

#include <sofa/component/misc/ReadState.h>
#include <sofa/component/misc/WriteState.h>

#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/FileRepository.h>


#ifdef SOFA_QT4
#include <QInputDialog>
#else
#include <qinputdialog.h>
#endif




namespace sofa
{

  namespace gui
  {

    namespace qt
    {


      using sofa::core::objectmodel::BaseObject;

      void RealGUI::clearRecord()
      {			
	timeSlider->setValue(0);	
	timeSlider->setMinValue(0);
	timeSlider->setMaxValue(0);		
	setRecordInitialTime(initial_time);
	setRecordFinalTime(initial_time);
	setRecordTime(initial_time);		  	
	//if ( viewer->getScene()) viewer->getScene()->execute< sofa::simulation::UpdateSimulationContextVisitor >();
	timeSlider->update();
      }
      
      bool RealGUI::setWriteSceneName()
      {	  
	bool ok;
	QString text = QInputDialog::getText("Record Simulation", "Enter the name of your simulation:", QLineEdit::Normal,
					     QString::null, &ok, this );
	if (!ok){return false;}
	if ( !text.isEmpty() ) {	    
	  std::string dir = sofa::helper::system::SetDirectory::GetParentDir(viewer->getSceneFileName().c_str()) ;
	  if (dir == "") dir = ".";
	  simulation_name = dir + "/" + text.ascii();
	  writeSceneName  = record_directory +  text.ascii();
	} else {
	  //Create the filename to use, to save the mechanical states
	  writeSceneName = record_directory+sofa::helper::system::SetDirectory::GetFileName(simulation_name.c_str());
	    std::string::size_type pos = writeSceneName.rfind('.');
	    if (pos != std::string::npos) writeSceneName.resize(pos);
	}
	return true;
      }

      void RealGUI::addReadState(bool init)
      {		
	sofa::component::misc::ReadStateCreator v(writeSceneName,false,init);
        v.addTag(core::objectmodel::Tag("AutoRecord"));
	v.execute(viewer->getScene());	
	std::cout << "Reading Recorded simulation with base name: " << writeSceneName << "\n";
      }
      
      void RealGUI::addWriteState()
      {	
	//record X, V, but won't record in the Mapping
	sofa::component::misc::WriteStateCreator v(writeSceneName, true, true, false);
        v.addTag(core::objectmodel::Tag("AutoRecord"));
	v.execute(viewer->getScene());	
	std::cout << "Recording simulation with base name: " << writeSceneName << "\n";
      }

      
      void RealGUI::slot_backward(  )
      {
	if (timeSlider->value() != timeSlider->minValue())
	{
	  setRecordTime(getRecordInitialTime());
	  slot_sliderValue(timeSlider->minValue());
	  loadSimulation();
	}
      }


      //-----------------------------------------------------------------------------------
      //Recording a simulation
      void RealGUI::slot_recordSimulation( bool value)
      {
	if (value) 
	{
	  Node* root = getScene();
	  if (root)
	  {	
 	   	 
	    if (setWriteSceneName())
	    {
	      clearRecord();
	      std::string filename = viewer->getSceneFileName();
		//Add if needed WriteState
		addWriteState();
		addReadState(false); //no init done		
		record_simulation=true;	
		playpauseGUI(true);
	      }
	    else  { record->setOn ( false );return;}
	  }
	}
	else 
	{
	  record_simulation=false;
	  playpauseGUI(false);
	    //Save simulation file

	  std::string filename = viewer->getSceneFileName();
	  filename = sofa::helper::system::SetDirectory::GetFileName(filename.c_str());
	  std::string simulationFileName = simulation_name + ".simu";

	  std::string output(record_directory + filename);
	    	    
	  std::ofstream out(simulationFileName.c_str());
	  if (!out.fail())
	  {
	    out << sofa::helper::system::DataRepository.getFile ( viewer->getSceneFileName() ) << " " << initialTime->text().ascii() << " " << finalTime->text().ascii() << " " << dtEdit->text().ascii() << " baseName: "<<writeSceneName;
	    out.close();
	  }
	  std::cout << "Simulation parameters saved in "<<simulationFileName<<std::endl;
	}
	  //Change the state of the writers
	sofa::component::misc::WriteStateActivator v_write(value);
        v_write.addTag(core::objectmodel::Tag("AutoRecord"));
	v_write.execute(viewer->getScene());	
	sofa::component::misc::ReadStateActivator v_read(false);
        v_read.addTag(core::objectmodel::Tag("AutoRecord"));
	v_read.execute(viewer->getScene());	  
      }
      
      void RealGUI::slot_stepbackward()
      {
	playforward_record->setOn(false);
	
	double init_time  = getRecordInitialTime();
	double time = getRecordTime() - atof(dtEdit->text());
	if (time < init_time) time = init_time;
	
	setRecordTime(time);
	slot_loadrecord_timevalue(false);
      }


      void RealGUI::slot_playforward( )
      {		
	if (playforward_record->isOn() ) 
	{
	  if (timeSlider->value() == timeSlider->maxValue())
	  {
	    playforward_record->setOn(false);
	  }
	  else
	  {
	    loadSimulation();
	    setPixmap("textures/media-playback-pause.png", playforward_record);
	    
	    timerRecordStep->start ( 0 );
	  }
	}	  
	else 
	{
	  timerRecordStep->stop();	  
	  setPixmap("textures/media-playback-start.png", playforward_record);
	}
      }

      void RealGUI::slot_stepforward()
      {
	if (timeSlider->value() != timeSlider->maxValue())
	{  
	  setRecordTime(getRecordTime() + atof(dtEdit->text()));	
	  slot_loadrecord_timevalue(false);
	}
      }

      void RealGUI::slot_forward(  )
      {
	if (timeSlider->value() != timeSlider->maxValue())
	{
	  setRecordTime(getRecordFinalTime());
	  slot_sliderValue(timeSlider->maxValue());	    
	}
      }

      void RealGUI::slot_sliderValue(int value, bool updateTime)
      {
	if (timeSlider->value() == value) return;
	double init_time   = getRecordInitialTime();
	double final_time  = getRecordFinalTime();
	if (updateTime)
	  {
	    double time = init_time + value/((float)timeSlider->maxValue())*(final_time-initial_time);
	    setRecordTime(time);
	  }
	timeSlider->setValue(value);
	timeSlider->update();
	loadSimulation();
      }

      void RealGUI::slot_loadrecord_timevalue(bool updateTime)
      {       
	
 	double init_time = getRecordInitialTime();
 	double final_time = getRecordFinalTime();
 	double current_time = getRecordTime();
	
	int value = (int)((current_time-init_time)/((float)(final_time-init_time))*timeSlider->maxValue());
		
	if (value > timeSlider->minValue())                                      slot_sliderValue(value, updateTime);
	else if ( value < timeSlider->maxValue())                                slot_sliderValue(value, updateTime);
	else if (!updateTime && value == timeSlider->maxValue())                 slot_sliderValue(value, updateTime);
	else if (value <= timeSlider->minValue()) 	                         slot_sliderValue(timeSlider->minValue());

	if (current_time >= final_time)
	{
	  playforward_record->setOn(false);
	  slot_sliderValue(timeSlider->maxValue());
	  setRecordTime(getRecordFinalTime());
 	  slot_playforward();
	}
      }

      //-------------------------------------------------
      //Given a direction  (forward, or backward), load the samples of the simulation between the initial and final time.
      void RealGUI::loadSimulation(bool one_step)
      {
	if (timeSlider->maxValue() == 0) 
	{
	  playforward_record->setOn(false);
	  return;
	}
		  
	float sleep_time = clock()/(float)CLOCKS_PER_SEC;
	double time=getRecordTime();
			
	//update the time in the context
	viewer->getScene()->execute< sofa::simulation::UpdateSimulationContextVisitor >();
	viewer->getScene()->execute< sofa::simulation::VisualUpdateVisitor >();
	
	//read the state for the current time
	sofa::component::misc::ReadStateModifier v(time);
        v.addTag(core::objectmodel::Tag("AutoRecord"));
	v.execute(viewer->getScene());		
		
	if (!one_step) sleep((viewer->getScene()->getDt()), sleep_time);	
		
	//Exporting sequence of OBJs 	
	if ( _animationOBJ )
	  {
#ifdef CAPTURE_PERIOD
	    static int counter = 0;
	    if ( ( counter++ % CAPTURE_PERIOD ) ==0 )
#endif
	      {
		exportOBJ ( false );
		++_animationOBJcounter;
	      }
	  }
	//Exporting states
	if ( m_dumpState )
	  simulation::getSimulation()->dumpState ( viewer->getScene(), *m_dumpStateStream );
	//Exporting in GNUplot format
	if ( m_exportGnuplot )
	  simulation::getSimulation()->exportGnuplot ( viewer->getScene(), viewer->getScene()->getTime() );

	emit newStep();	
        simulation::getSimulation()->getVisualRoot()->execute< sofa::simulation::UpdateMappingVisitor >();		 
      }


      //-------------------------------------------------
      double RealGUI::getRecordInitialTime() const
      {
	std::string init_time = initialTime->text().ascii();
	init_time.resize(init_time.size()-2);
	return fabs(atof((init_time.substr(6)).c_str()));
      }
      double RealGUI::getRecordFinalTime() const
      {
	std::string final_time = finalTime->text().ascii();
	final_time.resize(final_time.size()-2);
	return fabs(atof((final_time.substr(5)).c_str()));
      }
      double RealGUI::getRecordTime() const
      {
	return fabs(atof(loadRecordTime->text().ascii()));
      }
      void RealGUI::setRecordInitialTime(double time)
      {
	char buf[100];
	sprintf ( buf, "Init: %g s", fabs(time) );
	initialTime->setText ( buf ); 
      }
      void RealGUI::setRecordFinalTime(double time)
      {
	char buf[100];
	sprintf ( buf, "End: %g s", fabs(time) );
	finalTime->setText( buf );	
      }
      void RealGUI::setRecordTime(double time)
      {
	char buf[100];
	sprintf ( buf, "%g", fabs(time) );
	loadRecordTime->setText( buf );	   
	setTimeSimulation(getRecordTime());
      }
            
      void RealGUI::setTimeSimulation(double time)
      {
	char buf[100];
	sprintf ( buf, "Time: %g s", time );
	timeLabel->setText ( buf );	
	if (viewer->getScene())
	  viewer->getScene()->setTime(time);
      }
    } // namespace qt

  } // namespace gui

} // namespace sofa
