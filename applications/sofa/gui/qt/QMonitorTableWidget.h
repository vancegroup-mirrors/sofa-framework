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
#include "DataWidget.h"
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

      class QMonitorWidgetHelper : public QObject
      {
        Q_OBJECT
      public: 
        //QMonitorWidgetHelper(QObject* parent):QObject(parent){};
      protected slots:
        virtual void resizeTable(int) = 0;
      protected:
 
      };

      template <class DataTypes>
      class QMonitorWidget : 
        public TDataWidget<typename sofa::component::misc::Monitor<DataTypes>::MonitorData>
        , public QMonitorWidgetHelper
      {
        public:
          typedef typename TDataWidget<typename sofa::component::misc::Monitor<DataTypes>::MonitorData>::MyTData MyTData;
          QMonitorWidget(QWidget* parent,const char* name, MyTData* d):TDataWidget<typename sofa::component::misc::Monitor<DataTypes>::MonitorData>(parent,name,d){}//,QMonitorWidgetHelper(parent){};
          
          /* TDataWidget virtuals */ 
          virtual unsigned int sizeWidget(){return 3;}
          virtual unsigned int numColumnWidget(){return 4;}
          virtual bool createWidgets();
      protected:

        virtual void readFromData();
        virtual void writeToData();
        /* */ 
        /* QMonitorWidgetHelper virtuals */
        virtual void resizeTable(int);
        /* */
        void storeTable(std::list< std::pair< Q3Table*, core::objectmodel::BaseData*> >::iterator &it_listTable);
        
        Q3Table* createTableWidget(unsigned int sizeIdx);
        MyTData* data_;
        std::list< std::pair< Q3Table*, core::objectmodel::BaseData*> >  listTable_;
        std::map<QSpinBox*,Q3Table*> resizeMap_;
        std::set< Q3Table* > setResize_;
        Q3Table* vectorTable1_;
        Q3Table* vectorTable2_;
        Q3Table* vectorTable3_;


      };
    }
  }
}


#endif 


