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
      QMonitorTableWidget<DataTypes>::QMonitorTableWidget(typename QMonitorTableWidget<DataTypes>::TData* data, 
        const ModifyObjectFlags& flags, 
        QWidget* parent):QObjectMonitor(flags,parent),data_(data)
      {
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setResizeMode(QLayout::FreeResize);
        //internal monitorData
        typename Monitor<DataTypes>::MonitorData MonitorDataTemp = data_->getValue();
        //number of rows
        if (!MonitorDataTemp.sizeIdxPos() && !MonitorDataTemp.sizeIdxVels()
            && !MonitorDataTemp.sizeIdxForces() && !dialogFlags_.EMPTY_FLAG )
            return;

          layout->addWidget(new QLabel("", this));
          layout->addWidget(new QLabel("Positions", this));
          vectorTable1_ = addResizableTable(MonitorDataTemp.sizeIdxPos(),4);
          layout->addWidget(new QLabel (" ", this));

          vectorTable1_->setReadOnly(false);

          listTable_.push_back(std::make_pair(vectorTable1_, data_));

          vectorTable1_->horizontalHeader()->setLabel(0,QString("particle Indices"));
          vectorTable1_->setColumnStretchable(0,true);
          vectorTable1_->horizontalHeader()->setLabel(1,QString("X"));      
          vectorTable1_->setColumnStretchable(1,true);
          vectorTable1_->horizontalHeader()->setLabel(2,QString("Y"));      
          vectorTable1_->setColumnStretchable(2,true);
          vectorTable1_->horizontalHeader()->setLabel(3,QString("Z"));      
          vectorTable1_->setColumnStretchable(3,true);

          connect( vectorTable1_, SIGNAL( valueChanged(int,int) ), this, SLOT( UpdateWidget() ) );


          layout->addWidget(new QLabel("Velocities", this));
          vectorTable2_ = addResizableTable(MonitorDataTemp.sizeIdxVels(),4);
          layout->addWidget(new QLabel (" ", this));

          vectorTable2_->setReadOnly(false);

          listTable_.push_back(std::make_pair(vectorTable2_, data_));

          vectorTable2_->horizontalHeader()->setLabel(0,QString("particle Indices"));
          vectorTable2_->setColumnStretchable(0,true);
          vectorTable2_->horizontalHeader()->setLabel(1,QString("X"));
          vectorTable2_->setColumnStretchable(1,true);
          vectorTable2_->horizontalHeader()->setLabel(2,QString("Y"));
          vectorTable2_->setColumnStretchable(2,true);
          vectorTable2_->horizontalHeader()->setLabel(3,QString("Z"));
          vectorTable2_->setColumnStretchable(3,true);

          connect( vectorTable2_, SIGNAL( valueChanged(int,int) ), this, SLOT( UpdateWidget() ) );

          layout->addWidget(new QLabel("Forces", this));

          vectorTable3_ = addResizableTable(MonitorDataTemp.sizeIdxForces(),4);
          layout->addWidget(new QLabel (" ", this));
          vectorTable3_->setReadOnly(false);

          listTable_.push_back(std::make_pair(vectorTable3_, data_));

          vectorTable3_->horizontalHeader()->setLabel(0,QString("particle Indices"));
          vectorTable3_->setColumnStretchable(0,true);
          vectorTable3_->horizontalHeader()->setLabel(1,QString("X"));
          vectorTable3_->setColumnStretchable(1,true);
          vectorTable3_->horizontalHeader()->setLabel(2,QString("Y"));
          vectorTable3_->setColumnStretchable(2,true);
          vectorTable3_->horizontalHeader()->setLabel(3,QString("Z"));
          vectorTable3_->setColumnStretchable(3,true);

          connect( vectorTable3_, SIGNAL( valueChanged(int,int) ), this, SLOT( UpdateWidget() ) );

      }

      template <class DataTypes> 
      void QMonitorTableWidget<DataTypes>::update()
      {


        typename Monitor<DataTypes>::MonitorData MonitorDataTemp = data_->getValue();
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
        if (vectorTable1_ ) readOnlyData(vectorTable1_ ,data_);
        if (vectorTable2_) readOnlyData(vectorTable2_,data_);
        if (vectorTable3_) readOnlyData(vectorTable3_,data_);
        delete [] oss;
        delete [] oss2;
        delete [] oss3;

        counterWidget_ +=3;
        data_->setValue (MonitorDataTemp);
        emit TableValuesChanged(); //just in case we wanna intercept something.
      }

      template <class DataTypes>
      Q3Table * QMonitorTableWidget<DataTypes>::addResizableTable(const int& number,
        const int& column)
      {
        QVBoxLayout* layout = (QVBoxLayout*)this->layout();
        assert(layout != NULL);
        QSpinBox *spinBox = new QSpinBox(0,INT_MAX, 1, this);
        layout->addWidget(spinBox);
        Q3Table* table = new Q3Table(number,column, this);
        layout->addWidget(table);
        spinBox->setValue(number);
        resizeMap_.insert(std::make_pair(spinBox, table));
        connect( spinBox, SIGNAL( valueChanged(int) ), this, SLOT( resizeTable(int) ) );
        return  table;
      }

      template <class DataTypes>
      void QMonitorTableWidget<DataTypes>::storeTable(std::list< std::pair< Q3Table*, BaseData*> >::iterator &it_listTable)
      {
        Q3Table* table = it_listTable->first;
       
        //internal monitorData
        typename Monitor<DataTypes>::MonitorData NewMonitorData = data_->getValue();
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

        data_->setValue(NewMonitorData);
      }


      template <class DataTypes>
      void QMonitorTableWidget<DataTypes>::readOnlyData(Q3Table *widget, BaseData* data)
      {
        widget->setReadOnly(( (data->isReadOnly()) && dialogFlags_.READONLY_FLAG));
      }


    }
  }
}

