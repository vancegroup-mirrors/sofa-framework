#ifndef SOFA_GUI_QT_MODIFYOBJECTMODEL_H
#define SOFA_GUI_QT_MODIFYOBJECTMODEL_H

#include "SofaGUIQt.h"
#include <qglobal.h>

#ifdef SOFA_QT4
#include <QDialog>
#include <QWidget>
#include <Q3ListViewItem>
#else
#include <qdialog.h>
#include <qwidget.h>
#include <qlistview.h>
#endif // SOFA_QT4

#ifndef SOFA_QT4     
typedef QListViewItem Q3ListViewItem;
#endif


namespace sofa{
  namespace core{
    namespace objectmodel{
      class Base;
      class BaseData;
    }
  }
  namespace gui{
    namespace qt{

      union ObjectModel{
        sofa::core::objectmodel::BaseData* data;
        sofa::core::objectmodel::Base* base;
      };
    
      /*abstract class*/ 
      class SOFA_SOFAGUIQT_API ModifyObjectModel : public QDialog 
      {
        Q_OBJECT
      public:
         ModifyObjectModel( void *Id, 
           Q3ListViewItem* item_clicked, 
           QWidget* parent,
           const char* name= 0, 
           bool  modal= FALSE, 
           Qt::WFlags f= 0 );
      public slots:
        virtual void changeValue() = 0;
        virtual void updateValues() = 0;
        void reject   (){                 emit(dialogClosed(Id_)); deleteLater();QDialog::reject();} //When closing a window, inform the parent.
	      void accept   (){ updateValues(); emit(dialogClosed(Id_)); deleteLater();QDialog::accept();} //if closing by using Ok button, update the values
        signals:
        void objectUpdated();                 //update done
	      void dialogClosed(void *);            //the current window has been closed: we give the Id of the current window
      protected:
        void* Id_;
        Q3ListViewItem* item_;
        QWidget* parent_;


      };
    } //qt 
  } //gui
} //sofa

#endif //SOFA_GUI_QT_MODIFYOBJECTMODEL_H
