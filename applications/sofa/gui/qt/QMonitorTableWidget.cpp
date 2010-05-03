
#include "QMonitorTableWidget.inl"
#include <sofa/defaulttype/VecTypes.h>

using namespace sofa::component::misc;
namespace sofa{
  namespace gui{
    namespace qt{
      

#ifndef SOFA_DOUBLE
      template class QMonitorTableWidget<defaulttype::Vec3fTypes>;
#endif
#ifndef SOFA_FLOAT
      template class QMonitorTableWidget<defaulttype::Vec3dTypes>;
#endif

      QObjectMonitor::QObjectMonitor( const ModifyObjectFlags& flags,QWidget* parent):
        QWidget(parent),
        vectorTable1_(NULL),
        vectorTable2_(NULL),
        vectorTable3_(NULL),
        counterWidget_(0),
        dialogFlags_(flags)
        {
        }
      void QObjectMonitor::resizeTable(int number)
      {
        QSpinBox *spinBox = (QSpinBox *) sender();
        Q3Table *table = resizeMap_[spinBox];
        if (number != table->numRows())
        {
          table->setNumRows(number);
          setResize_.insert(table);
          UpdateWidget();
        }
      }

      void QObjectMonitor::UpdateData()
      {
        std::list< std::pair< Q3Table*, BaseData*> >::iterator it_listTable;
        for (it_listTable = listTable_.begin(); it_listTable != listTable_.end(); it_listTable++)
        {
          storeTable(it_listTable);
        }
      }

      void QObjectMonitor::UpdateWidget()
      {
        update();
      }
    }
  }
}
