
#include "QMonitorTableWidget.inl"
#include <sofa/defaulttype/VecTypes.h>

using namespace sofa::component::misc;
namespace sofa{
  namespace gui{
    namespace qt{
      

#ifndef SOFA_DOUBLE
      template class QMonitorWidget<defaulttype::Vec3fTypes>;
            helper::Creator<DataWidgetFactory, QMonitorWidget<defaulttype::Vec3fTypes> > DWClass_Monitor3f("default",true);
#endif
#ifndef SOFA_FLOAT
      template class QMonitorWidget<defaulttype::Vec3dTypes>;
      helper::Creator<DataWidgetFactory, QMonitorWidget<defaulttype::Vec3dTypes> > DWClass_Monitor3d("default",true);
#endif
    }
  }
}

