/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/core/objectmodel/ConfigurationSetting.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

ConfigurationSetting::ConfigurationSetting()
{
    this->name.setDisplayed(false);        this->name.setPersistent(false);
    this->f_printLog.setDisplayed(false);  this->f_printLog.setPersistent(false);
    this->f_tags.setDisplayed(false);      this->f_tags.setPersistent(false);
    this->f_listening.setDisplayed(false); this->f_listening.setPersistent(false);
}

ConfigurationSetting::~ConfigurationSetting()
{
}   

void ConfigurationSetting::init()
{
  //Set all the Data in read only mode:
  std::vector< std::pair<std::string, BaseData*> >::iterator it;
  for (it=m_fieldVec.begin();it!=m_fieldVec.end();++it)
  {
    BaseData* data=it->second;
    data->setReadOnly(true);
  }
}

} // namespace objectmodel

} // namespace core

} // namespace sofa
