#ifndef SOFA_GUI_QT_QMONITORTABLEWIDGET_H
#define SOFA_GUI_QT_QMONITORTABLEWIDGET_H

#ifdef SOFA_QT4
#include <QWidget>
#include <Q3Table>
#include <QLabel>
#include <QSpinBox>
#else
#include <qwidget.h>
#include <qtable.h>
#include <qlabel.h>
#include <qspinbox.h>
#endif

#include <sofa/component/misc/Monitor.h>
#include <sofa/defaulttype/Vec.h>
#include <list>

#ifndef SOFA_QT4     
typedef QTable    Q3Table;
#endif

namespace sofa{
  namespace core{
    namespace objectmodel{
      class BaseData;
    }
  }
  namespace gui{
    namespace qt{


      struct ModifyObjectFlags;
      class QObjectMonitor : public QWidget
      {
        Q_OBJECT
  
     public slots:
        void UpdateWidget();
        void UpdateData();
     signals: 
        void TableValuesChanged();
        void TablesNeedUpdate(); 
      protected slots:
        void resizeTable(int number);

      public: 
        QObjectMonitor(const ModifyObjectFlags& flags ,
          QWidget* parent = 0);
 
      protected:
        virtual void update() = 0; // were the actual stuff is done when UpdateTables is called.
        virtual void storeTable(std::list< std::pair< Q3Table*, core::objectmodel::BaseData*> >::iterator &it_listTable) = 0;
        //where the actual stuff is done when saveTables is called.
        

        Q3Table* vectorTable1_;
        Q3Table* vectorTable2_;
        Q3Table* vectorTable3_;
        unsigned int counterWidget_;
        const ModifyObjectFlags& dialogFlags_;
        std::map<QSpinBox*,Q3Table*> resizeMap_;
        std::set< Q3Table* > setResize_;
        std::list< std::pair< Q3Table*, core::objectmodel::BaseData*> >  listTable_;
        
      };
   
      template <class DataTypes>
      class QMonitorTableWidget : public QObjectMonitor
      {
        typedef Data<typename sofa::component::misc::Monitor<DataTypes>::MonitorData > TData;
    
      protected:
        virtual void storeTable(std::list< std::pair< Q3Table*, core::objectmodel::BaseData*> >::iterator &it_listTable);
        virtual void update();
        Q3Table* addResizableTable(const int& number, const int& column);
        void readOnlyData(Q3Table *widget, core::objectmodel::BaseData* data);
        TData* data_;
        unsigned int numWidgets_;
      public:
        QMonitorTableWidget(TData* data,const ModifyObjectFlags& flags,QWidget* parent);
      };
    }
  }
}


#endif 
