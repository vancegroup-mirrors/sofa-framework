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
#ifndef SOFA_GUI_QT_MODIFYDATA_H
#define SOFA_GUI_QT_MODIFYDATA_H

#include "SofaGUIQt.h"
#include "ModifyObjectModel.h"
#include <qglobal.h>

#ifdef SOFA_QT4
#include <QDialog>
#include <QWidget>
#include <Q3ListViewItem>
#else
#include <qdialog.h>
#include <qwidget.h>
#include <qlistview.h>
#endif // SOFA_QT4

#ifndef SOFA_QT4     
typedef QListViewItem Q3ListViewItem;
#endif




namespace sofa{
  namespace core{
    namespace objectmodel{
      class BaseData;
    }
  }
  namespace gui{
    namespace qt{

      class SOFA_SOFAGUIQT_API ModifyData : public ModifyObjectModel
      {
        Q_OBJECT
      public:
        ModifyData(void* Id,
          core::objectmodel::BaseData* data,
          Q3ListViewItem* item,
          QWidget* parent,
          const char* name= 0, 
          bool  modal= FALSE, 
          Qt::WFlags f= 0 );
        public slots:
          virtual void changeValue(){};
          virtual void updateValues(){};
      protected:
        core::objectmodel::BaseData* data_;
        Q3ListViewItem* item_;
        QWidget* parent_;
      };
    
    

    
    } // qt 
  } // gui
} // sofa

#endif // SOFA_GUI_QT_MODIFYDATA_H
