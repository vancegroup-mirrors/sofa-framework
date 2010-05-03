#include "ModifyData.h"
#include "DataWidget.h"
#include <sofa/core/objectmodel/BaseData.h>
namespace sofa{
  using namespace core::objectmodel;
  namespace gui{
    namespace qt{
   
      ModifyData::ModifyData(void* Id,
          BaseData* data,
          Q3ListViewItem* item,
          QWidget* parent,
          const char* name, 
          bool  modal, 
          Qt::WFlags f):ModifyObjectModel(Id,item,parent,name,modal,f),
          data_(data),item_(item),parent_(parent)
      {
      }

    } //qt
  } //gui
} //sofa
