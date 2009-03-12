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

#include <sofa/gui/qt/DisplayFlagWidget.h>

namespace sofa
{

  namespace gui
  {

    namespace qt
    {

      DisplayFlagWidget::DisplayFlagWidget(QWidget* parent, const char* name,  Qt::WFlags f ):
	Q3ListView(parent,name,f)
      {
	addColumn(QString::null);
	setRootIsDecorated( TRUE );
	setTreeStepSize( 12 );
	header()->hide();
	clear();

	setMouseTracking(false);

#ifdef SOFA_QT4
	setFocusPolicy(Qt::NoFocus);
#else
	setFocusPolicy(QWidget::NoFocus);
#endif
	
	setFrameShadow(QFrame::Plain);
	setFrameShape(QFrame::NoFrame );

	setSortColumn(-1);
	Q3CheckListItem* itemShowAll = new Q3CheckListItem(this, "All", Q3CheckListItem::CheckBoxController);
	itemShowAll->setOpen(true);
	Q3CheckListItem* itemShowVisual    = new Q3CheckListItem(itemShowAll, "Visual", Q3CheckListItem::CheckBoxController);
	itemShowVisual->setOpen(true);
	itemShowFlag[VISUAL]   = new Q3CheckListItem(itemShowVisual, "Visual Models", Q3CheckListItem::CheckBox);
	Q3CheckListItem* itemShowBehavior  = new Q3CheckListItem(itemShowAll, itemShowVisual, "Behavior", Q3CheckListItem::CheckBoxController);
	itemShowBehavior->setOpen(true);
	itemShowFlag[BEHAVIOR]   = new Q3CheckListItem(itemShowBehavior,  "Behavior Models", Q3CheckListItem::CheckBox);
	itemShowFlag[FORCEFIELD]   = new Q3CheckListItem(itemShowBehavior, itemShowFlag[BEHAVIOR], "Force Fields", Q3CheckListItem::CheckBox);
	itemShowFlag[INTERACTION]   = new Q3CheckListItem(itemShowBehavior, itemShowFlag[FORCEFIELD],  "Interactions", Q3CheckListItem::CheckBox);
	Q3CheckListItem* itemShowCollision = new Q3CheckListItem(itemShowAll, itemShowBehavior, "Collision", Q3CheckListItem::CheckBoxController);
	itemShowCollision->setOpen(true);
	itemShowFlag[COLLISION]   = new Q3CheckListItem(itemShowCollision,  "Collision Models", Q3CheckListItem::CheckBox);
	itemShowFlag[BOUNDING]   = new Q3CheckListItem(itemShowCollision, itemShowFlag[COLLISION], "Bounding Trees", Q3CheckListItem::CheckBox);
	Q3CheckListItem* itemShowMapping   = new Q3CheckListItem(itemShowAll, itemShowCollision, "Mapping", Q3CheckListItem::CheckBoxController);
	itemShowMapping->setOpen(true);
	itemShowFlag[MAPPING]   = new Q3CheckListItem(itemShowMapping,  "Visual Mappings", Q3CheckListItem::CheckBox);
	itemShowFlag[MECHANICALMAPPING]   = new Q3CheckListItem(itemShowMapping, itemShowFlag[MAPPING],  "Mechanical Mappings", Q3CheckListItem::CheckBox);
	Q3ListViewItem*  itemShowOptions   = new Q3ListViewItem(this, itemShowAll, "Options");
	itemShowOptions->setOpen(true);
	itemShowFlag[WIREFRAME]   = new Q3CheckListItem(itemShowOptions, "Wire Frame", Q3CheckListItem::CheckBox);
	itemShowFlag[NORMALS]   = new Q3CheckListItem(itemShowOptions, itemShowFlag[WIREFRAME], "Normals", Q3CheckListItem::CheckBox);

	insertItem(itemShowAll); 
	itemShowAll->insertItem(itemShowVisual); itemShowAll->setOpen(true);
	itemShowVisual->insertItem(itemShowFlag[VISUAL]);
	itemShowAll->insertItem(itemShowBehavior); 
	itemShowBehavior->insertItem(itemShowFlag[BEHAVIOR]);	
	itemShowBehavior->insertItem(itemShowFlag[FORCEFIELD]);
	itemShowBehavior->insertItem(itemShowFlag[INTERACTION]);
	itemShowAll->insertItem(itemShowCollision); 
	itemShowCollision->insertItem(itemShowFlag[COLLISION]);
	itemShowCollision->insertItem(itemShowFlag[BOUNDING]);
	itemShowAll->insertItem(itemShowMapping); 
	itemShowMapping->insertItem(itemShowFlag[MAPPING]);
	itemShowMapping->insertItem(itemShowFlag[MECHANICALMAPPING]);
			  
	insertItem(itemShowOptions); itemShowOptions->setOpen(true);
	itemShowOptions->insertItem(itemShowFlag[WIREFRAME]);
	itemShowOptions->insertItem(itemShowFlag[NORMALS]);

 	for (int i=0;i<10;++i)  mapFlag.insert(std::make_pair(itemShowFlag[i],i));
      }

      void DisplayFlagWidget::findChildren(Q3CheckListItem *item, std::vector<Q3CheckListItem* > &children)
      {	
	Q3CheckListItem * child = (Q3CheckListItem * )item->firstChild();
	while(child) {
	  children.push_back(child);
	  findChildren(child,children);
	  child = (Q3CheckListItem * )child->nextSibling();
	}
      }


      void DisplayFlagWidget::contentsMousePressEvent ( QMouseEvent * e )
	{

	  if ( Q3CheckListItem *item = dynamic_cast<Q3CheckListItem *>(itemAt(contentsToViewport(e->pos()))) )
	    {
	      std::vector< Q3CheckListItem *> childDepending;
	      findChildren(item, childDepending);

	      bool value=!item->isOn();
	      item->setOn(value);
 
	      if (mapFlag.find(item) != mapFlag.end()) emit change(mapFlag[item],value);
	      for (unsigned int idxChild=0;idxChild<childDepending.size();++idxChild)
		{
		  if (mapFlag.find(childDepending[idxChild]) != mapFlag.end()) emit change(mapFlag[childDepending[idxChild]],value);
		}
	      emit clicked();
	    }
	}

    } // namespace qt

  } // namespace gui

} // namespace sofa

