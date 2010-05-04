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
#include <sofa/gui/qt/TrickyDataWidget.h>


namespace sofa
{

namespace gui
{

namespace qt
{

//#ifdef SOFA_DEV
//#include <sofa/helper/tricks.h>

helper::Creator<DataWidgetFactory,RadioDataWidget> DWClass_RadioTrick("default",true);

bool RadioDataWidget::createWidgets()
{
	buttonList=new QButtonGroup(this);

  QVBoxLayout* layout = new QVBoxLayout(this);
  /*=====================================================================================*/
  sofa::helper::RadioTrick<std::string> m_radiotrick = getData()->virtualGetValue();
  for(unsigned int i=0;i<m_radiotrick.size();i++)
  {
	  std::string m_itemstring=m_radiotrick[i];

	  QRadioButton * m_radiobutton=new QRadioButton(QString::fromStdString(m_itemstring));
	  if (i==m_radiotrick.getSelectedId()) m_radiobutton->setChecked(true);
	  layout->add(m_radiobutton);
	  buttonList->addButton(m_radiobutton,i);
  }
  /*=====================================================================================*/

  connect(buttonList, SIGNAL(buttonClicked(int)), this, SLOT(setbuttonchekec(int))) ;
  return true;
}

void RadioDataWidget::readFromData()
{
	 // sofa::helper::RadioTrick m_radiotrick = getData()->virtualGetValue();
}

void RadioDataWidget::writeToData()
{
	 // sofa::helper::RadioTrick m_radiotrick = getData()->virtualGetValue();

}

void RadioDataWidget::change()
{

}
void RadioDataWidget::setbuttonchekec(int id_checked)
{
  sofa::helper::RadioTrick<std::string> m_radiotrick = getData()->virtualGetValue();
	m_radiotrick.setSelectedItem((unsigned int)id_checked);
	//std::cout<<"=============================checked number :"<<id_checked<<std::endl;
}




//#endif


} // namespace qt

} // namespace gui

} // namespace sofa


