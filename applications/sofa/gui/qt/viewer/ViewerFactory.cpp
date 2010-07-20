#include <sofa/gui/qt/viewer/ViewerFactory.h>
#include <sofa/helper/Factory.inl>

namespace sofa{

  namespace helper {

    template class sofa::helper::Factory<std::string, sofa::gui::qt::viewer::SofaViewer, sofa::gui::qt::viewer::CreatorArgument>;

  }
}
