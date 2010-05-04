#include "QMonitorTableWidget.h"
#include "ModifyObject.h"
#include <sofa/helper/vector.h>
#include <iostream>

#ifdef SOFA_QT4
#include <QVBoxLayout>
#else
#include <qlayout.h>
#endif



namespace sofa{
  using namespace sofa::component::misc;
  using namespace sofa::core::objectmodel;
  namespace gui{
    namespace qt{

      template <class DataTypes>
      bool QMonitorWidget<DataTypes>::createWidgets()
      {
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setResizeMode(QLayout::FreeResize);
        //internal monitorData
        typename Monitor<DataTypes>::MonitorData MonitorDataTemp = this->getData()->virtualGetValue();
        //number of rows
        if (!MonitorDataTemp.sizeIdxPos() && !MonitorDataTemp.sizeIdxVels()
          && !MonitorDataTemp.sizeIdxForces())
          return false;

        layout->addWidget(new QLabel("", this));
        layout->addWidget(new QLabel("Positions", this));
        vectorTable1_ = createTableWidget(MonitorDataTemp.sizeIdxPos());
        layout->addWidget(new QLabel (" ", this));

        layout->addWidget(new QLabel("Velocities", this));
        vectorTable2_ = createTableWidget(MonitorDataTemp.sizeIdxVels());
        layout->addWidget(new QLabel (" ", this));

        layout->addWidget(new QLabel("Forces", this));
        vectorTable3_ = createTableWidget(MonitorDataTemp.sizeIdxForces());
        layout->addWidget(new QLabel (" ", this));

        return true;
      }

      template <class DataTypes>
      Q3Table* QMonitorWidget<DataTypes>::createTableWidget(unsigned int sizeIdx)
      {
        QSpinBox *spinBox = new QSpinBox(0,INT_MAX, 1, this);
        this->layout()->add(spinBox);
        Q3Table* table = new Q3Table(sizeIdx,4, this);
        this->layout()->add(table);
        spinBox->setValue(sizeIdx);
        resizeMap_.insert(std::make_pair(spinBox, table));
        listTable_.push_back( std::make_pair(table, this->getData()) );


        table->setReadOnly(false);
        table->horizontalHeader()->setLabel(0,QString("particle Indices"));
        table->setColumnStretchable(0,true);
        table->horizontalHeader()->setLabel(1,QString("X"));      
        table->setColumnStretchable(1,true);
        table->horizontalHeader()->setLabel(2,QString("Y"));      
        table->setColumnStretchable(2,true);
        table->horizontalHeader()->setLabel(3,QString("Z"));      
        table->setColumnStretchable(3,true);

        /* 
        as this class inherits from QMonitorWidgetHelper and TDataWidget 
        which both derives from QObject,some explicit disambiguation must be done...
        */
        QMonitorWidgetHelper::connect( spinBox, SIGNAL( valueChanged(int) ), (QMonitorWidgetHelper*)this, SLOT( resizeTable(int) ) );
        QMonitorWidgetHelper::connect( spinBox, SIGNAL( valueChanged(int) ), (DataWidget*)this, SLOT( updateWidgetValue() ) );
        QMonitorWidgetHelper::connect( spinBox, SIGNAL( valueChanged(int) ), (DataWidget*)this, SLOT( updateWidgetValue() ) );
        QMonitorWidgetHelper::connect( table,   SIGNAL( valueChanged(int,int) ), (DataWidget*)this, SLOT( updateWidgetValue() ) );
        return table;
      }

      template <class DataTypes>
      void QMonitorWidget<DataTypes>::resizeTable(int value)
      {
        QSpinBox *spinBox = (QSpinBox *) QMonitorWidgetHelper::sender();
        if( spinBox == NULL){
          return;
        }
        Q3Table *table = resizeMap_[spinBox];
        if (value != table->numRows())
        {
          table->setNumRows(value);
          setResize_.insert(table);
        }
      }

      template <class DataTypes> 
      void QMonitorWidget<DataTypes>::writeToData()
      {
        std::list< std::pair< Q3Table*, BaseData*> >::iterator it_listTable;
        for (it_listTable = listTable_.begin(); it_listTable != listTable_.end(); it_listTable++)
        {
          storeTable(it_listTable);
        }
      }

      template <class DataTypes> 
      void QMonitorWidget<DataTypes>::readFromData()
      {

        typename Monitor<DataTypes>::MonitorData MonitorDataTemp = this->getData()->virtualGetValue();
        //number of rows
        unsigned short int nbRowVels = 0, nbRowForces = 0, nbRowPos = 0;
        //number of rows for positions
        if (MonitorDataTemp.getSizeVecPos())
        {
          if (setResize_.find(vectorTable1_) != setResize_.end())
          {
            sofa::helper::vector < int > NewIndPos;
            NewIndPos = MonitorDataTemp.getIndPos();
            NewIndPos.resize(vectorTable1_->numRows(), 0);
            nbRowPos = NewIndPos.size();
            MonitorDataTemp.setIndPos (NewIndPos);
          }
          else
          {
            nbRowPos = MonitorDataTemp.sizeIdxPos();
            vectorTable1_->setNumRows(nbRowPos);
          }
        }
        else
        {
          vectorTable1_->setNumRows(nbRowPos);
        }

        //number of rows for velocities
        if (MonitorDataTemp.getSizeVecVels())
        {
          if (setResize_.find(vectorTable2_) != setResize_.end())
          {
            sofa::helper::vector < int > NewIndVels;
            NewIndVels = MonitorDataTemp.getIndVels();
            NewIndVels.resize(vectorTable2_->numRows(), 0);
            nbRowVels = NewIndVels.size();
            MonitorDataTemp.setIndVels (NewIndVels);
          }
          else
          {
            nbRowVels = MonitorDataTemp.sizeIdxVels();
            vectorTable2_->setNumRows(nbRowVels);
          }
        }
        else
        {
          vectorTable2_->setNumRows(nbRowVels);
        }

        //number of rows for forces
        if (MonitorDataTemp.getSizeVecForces())
        {
          if (setResize_.find(vectorTable3_) != setResize_.end())
          {
            sofa::helper::vector < int > NewIndForces;
            NewIndForces = MonitorDataTemp.getIndForces();
            NewIndForces.resize(vectorTable3_->numRows(), 0);
            nbRowForces = NewIndForces.size();
            MonitorDataTemp.setIndForces (NewIndForces);
          }
          else
          {
            nbRowForces = MonitorDataTemp.sizeIdxForces();
            vectorTable3_->setNumRows(nbRowForces);
          }
        }
        else
        {
          vectorTable3_->setNumRows(nbRowForces);
        }

        setResize_.clear();


        for (unsigned int i=0; i<3; i++)
        {
          std::ostringstream *oss = new std::ostringstream[nbRowPos];
          for (unsigned int j=0; j<nbRowPos; j++)
          {
            oss[j] << (MonitorDataTemp.getPos(j))[i];
            vectorTable1_->setText(j,i+1,std::string(oss[j].str()).c_str());
          }

          std::ostringstream * oss2 = new std::ostringstream[nbRowVels];
          for (unsigned int j=0; j<nbRowVels; j++)
          {
            oss2[j] << (MonitorDataTemp.getVel(j))[i];
            vectorTable2_->setText(j,i+1,std::string(oss2[j].str()).c_str());
          }

          std::ostringstream * oss3 = new std::ostringstream[nbRowForces];
          for (unsigned int j=0; j<nbRowForces; j++)
          {
            oss3[j] << (MonitorDataTemp.getForce(j))[i];
            vectorTable3_->setText(j,i+1,std::string(oss3[j].str()).c_str());
          }
          delete [] oss;
          delete [] oss2;
          delete [] oss3;
        }
        //vectorTable1
        std::ostringstream  * oss = new std::ostringstream[nbRowPos];
        for (unsigned int j=0; j<nbRowPos; j++)
        {
          oss[j] << MonitorDataTemp.getIndPos()[j];
          vectorTable1_->setText(j,0,std::string(oss[j].str()).c_str());
        }
        //vectorTable2_
        std::ostringstream * oss2 = new std::ostringstream[nbRowVels];
        for (unsigned int j=0; j<nbRowVels; j++)
        {
          oss2[j] << MonitorDataTemp.getIndVels()[j];
          vectorTable2_->setText(j,0,std::string(oss2[j].str()).c_str());
        }
        //vectorTable3_
        std::ostringstream * oss3 = new std::ostringstream[nbRowForces];
        for (unsigned int j=0; j<nbRowForces; j++)
        {
          oss3[j] << MonitorDataTemp.getIndForces()[j];
          vectorTable3_->setText(j,0,std::string(oss3[j].str()).c_str());
        }
        delete [] oss;
        delete [] oss2;
        delete [] oss3;

        this->getData()->virtualSetValue(MonitorDataTemp);
      }


      //writeToData()
      template <class DataTypes>
      void QMonitorWidget<DataTypes>::storeTable(std::list< std::pair< Q3Table*, BaseData*> >::iterator &it_listTable)
      {
        Q3Table* table = it_listTable->first;

        //internal monitorData
        typename Monitor<DataTypes>::MonitorData NewMonitorData = this->getData()->virtualGetValue();
        //Qtable positions
        if (NewMonitorData.getSizeVecPos())
        {
          int valueBox;
          sofa::helper::vector < int > values = NewMonitorData.getIndPos();
          for (int i=0; i < table -> numRows(); i++)
          {
            valueBox = (int)atof(table->text(i,0));
            if(valueBox >= 0 && valueBox <= (int)(NewMonitorData.getSizeVecPos() - 1))
              values[i] = valueBox;
          }

          NewMonitorData.setIndPos(values);
        }
        it_listTable++;
        table = it_listTable->first;

        //Qtable velocities

        if (NewMonitorData.getSizeVecVels())
        {
          int valueBox;
          sofa::helper::vector < int > values = NewMonitorData.getIndVels();
          for (int i=0; i < table -> numRows(); i++)
          {
            valueBox = (int)atof(table->text(i,0));
            if(valueBox >= 0 && valueBox <= (int)(NewMonitorData.getSizeVecVels() - 1))
              values[i] = valueBox;
          }

          NewMonitorData.setIndVels(values);
        }
        it_listTable++;
        table=it_listTable->first;


        //Qtable forces

        if (NewMonitorData.getSizeVecForces())
        {
          int valueBox;
          sofa::helper::vector < int > values = NewMonitorData.getIndForces();

          for (int i=0; i < table -> numRows(); i++)
          {
            valueBox = (int)atof(table->text(i,0));
            if(valueBox >= 0 && valueBox <= (int)(NewMonitorData.getSizeVecForces() - 1))
              values[i] = valueBox;
          }

          NewMonitorData.setIndForces(values);
        }

        this->getData()->virtualSetValue(NewMonitorData);
      }
    }
  }
}

