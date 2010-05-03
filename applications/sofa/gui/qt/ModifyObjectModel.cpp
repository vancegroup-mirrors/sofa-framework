#include "ModifyObjectModel.h"
#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseData.h>

namespace sofa{
  namespace gui{
    namespace qt{

      ModifyObjectModel::ModifyObjectModel(void* Id,
        Q3ListViewItem* item_clicked,
        QWidget* parent,
        const char* name, 
        bool  modal, 
        Qt::WFlags f):QDialog(parent, name, modal, f),
        Id_(Id),item_(item_clicked),parent_(parent)
      {
        	//Title of the Dialog
	      setCaption(name);
        connect ( this, SIGNAL( objectUpdated() ), parent_, SLOT( redraw() ));
	      connect ( this, SIGNAL( dialogClosed(void *) ) , parent_, SLOT( modifyUnlock(void *)));
      }
    }
  }
}
