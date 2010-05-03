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
#ifndef SOFA_GUI_QT_QMOUSEOPERATIONS_H
#define SOFA_GUI_QT_QMOUSEOPERATIONS_H
 
#include <sofa/gui/MouseOperations.h>
#include <sofa/gui/qt/SofaMouseManager.h>
#ifdef SOFA_QT4
#include <QWidget>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QSlider>
#else
#include <qwidget.h>
#include <qlineedit.h>
#include <qradiobutton.h>
#include <qspinbox.h>
#include <qslider.h>
#endif
#include <iostream>

namespace sofa
{

  namespace gui
  {

    namespace qt
    {


      class QAttachOperation : public QWidget, public AttachOperation
      {
        Q_OBJECT
      public:
        QAttachOperation();
        
        double getStiffness() const;
        void configure(PickHandler *picker, MOUSE_BUTTON b)
        {
          AttachOperation::configure(picker, b);
        }
      protected:
        QLineEdit *value;
      };

      
      
      class QInciseOperation : public QWidget, public InciseOperation
      {
	Q_OBJECT
      public:
        QInciseOperation();
        int getIncisionMethod() const;
	int getSnapingBorderValue() const;
	int getSnapingValue() const;
	
        void configure(PickHandler *picker, MOUSE_BUTTON b)
        {
          InciseOperation::configure(picker, b);
        }

      protected:
	QGroupBox* incisionMethodChoiceGroup;
	QRadioButton* method1;
	QRadioButton* method2;

	QGroupBox* advancedOptions;
	QSlider  *snapingBorderSlider;
	QSpinBox *snapingBorderValue;

	QSlider  *snapingSlider;
	QSpinBox *snapingValue;
      };


      
      class QFixOperation : public QWidget, public FixOperation
      {
        Q_OBJECT
      public:
        QFixOperation();
        double getStiffness() const;
        void configure(PickHandler *picker, MOUSE_BUTTON b)
        {
          FixOperation::configure(picker, b);
        }

      protected:
        QLineEdit *value;
      };


      
      class QTopologyOperation : public QWidget, public TopologyOperation
      {
        Q_OBJECT
      public:
        QTopologyOperation();
	double getScale() const;
	int getTopologicalOperation() const;
	bool getVolumicMesh() const;
	
	void configure(PickHandler *picker, MOUSE_BUTTON b)
        {
          TopologyOperation::configure(picker, b);
        }

   public slots:
   void setEnableBox (int i);

      protected:

	QComboBox *operationChoice;

	QGroupBox *advancedOptions;
	QRadioButton *meshType1;
	QRadioButton *meshType2;

	QSlider *scaleSlider;
	QSpinBox *scaleValue;
      };
	

      
      class QInjectOperation : public QWidget, public InjectOperation
      {
        Q_OBJECT
      public:
        QInjectOperation();
        double getPotentialValue() const;
	std::string getStateTag() const;
        void configure(PickHandler *picker, MOUSE_BUTTON b)
        {
          InjectOperation::configure(picker, b);
        }

      protected:
        QLineEdit *value;
	QLineEdit *tag;
      };

    }
  }
}

#endif
