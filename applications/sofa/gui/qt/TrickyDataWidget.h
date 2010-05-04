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
#ifndef SOFA_GUI_QT_TrickyDATAWIDGET_H
#define SOFA_GUI_QT_TrickyDATAWIDGET_H

#include <sofa/gui/qt/DataWidget.h>
//#include <sofa/gui/qt/ModifyObject.h>

#ifdef SOFA_QT4
#include <QLabel>
#include <QVBoxLayout>
#include <QSlider>
#include <QString>
#else
#include <qlabel.h>
#include <qlayout.h>
#include <qslider.h>
#include <qstring.h>
#endif

#include <sofa/component/fem/QuadratureFormular.h>
#include <sofa/helper/Polynomial_LD.inl>

#include <sofa/helper/tricks.h>



namespace sofa
{

namespace gui
{

namespace qt
{

class RadioDataWidget : public TDataWidget<sofa::helper::RadioTrick<std::string> >
{
  Q_OBJECT
public :

  ///The class constructor takes a TData<RadioTrick> since it creates
  ///a widget for a that particular data type.
  RadioDataWidget(QWidget* parent, const char* name,
    core::objectmodel::TData<sofa::helper::RadioTrick<std::string> >* m_data)
    : TDataWidget<sofa::helper::RadioTrick<std::string> >(parent,name,m_data){};

  ///In this method we  create the widgets and perform the signal / slots connections.
  virtual bool createWidgets();

protected slots:
  void change();
  void setbuttonchekec(int id_checked);
protected:
  ///Implements how update the widgets knowing the data value.
  virtual void readFromData();
  ///Implements how to update the data, knowing the widget value.
  virtual void writeToData();
  QButtonGroup *buttonList;

};


} // namespace qt

} // namespace gui

} // namespace sofa


#endif
