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

#include "QCategoryLibrary.h"
#include "QComponentLibrary.h"

namespace sofa
{

  namespace gui
  {

    namespace qt
    {


      QCategoryLibrary::QCategoryLibrary( QWidget *parent, const std::string &categoryName, unsigned int numComponent): QWidget(parent, categoryName.c_str()), CategoryLibrary(categoryName)
      {    
        layout = new CategoryLayout( this, numComponent ); 
      }
      
      QCategoryLibrary::~QCategoryLibrary()
      {
        for (unsigned int i=0;i<components.size();++i)
          {
            delete components[i];
          }
        delete layout;
        components.clear();
      }
      ComponentLibrary *QCategoryLibrary::createComponent(const std::string &componentName, ClassEntryPtr& entry, const std::vector< std::string > &exampleFiles)
      {        
        QComponentLibrary* component = new QComponentLibrary(this, layout, componentName, this->getName(), entry, exampleFiles); 
        return component;
      }

      ComponentLibrary *QCategoryLibrary::addComponent(const std::string &componentName, ClassEntryPtr& entry, const std::vector< std::string > &exampleFiles)
      {  
        QComponentLibrary *component = static_cast<QComponentLibrary *>(CategoryLibrary::addComponent(componentName, entry, exampleFiles));
        if (component)
          {            
            layout->addWidget(component, components.size()-1,0);
            connect( component->getQWidget(), SIGNAL( componentDragged( std::string, std::string, ClassEntryPtr& ) ),
                     this, SLOT( componentDraggedReception( std::string, std::string, ClassEntryPtr&) ) );
          }
        return component;
      }


      void QCategoryLibrary::endConstruction()
      {       
        layout->addItem(new QSpacerItem(1,1,QSizePolicy::Minimum, QSizePolicy::Expanding ), layout->numRows(),0);
      }


      void QCategoryLibrary::setDisplayed(bool b)
      {       
        if (b) this->show();
        else   this->hide();
      }


      //*********************//
      // SLOTS               //
      //*********************//
      void QCategoryLibrary::componentDraggedReception( std::string description, std::string templateName, ClassEntryPtr& componentEntry)
      {
        emit( componentDragged( description, this->getName(), templateName, componentEntry) );
      }

    }
  }
}
