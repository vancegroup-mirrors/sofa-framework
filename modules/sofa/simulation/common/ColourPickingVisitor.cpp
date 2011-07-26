#include <sofa/simulation/common/ColourPickingVisitor.h>
#include <sofa/helper/system/gl.h>

namespace sofa{
  namespace simulation{


    Visitor::Result ColourPickingVisitor::processNodeTopDown(simulation::Node* node)
    {
      glPushMatrix();
#ifdef SOFA_SUPPORT_MOVING_FRAMES
            double glMatrix[16];
	    node->getPositionInWorld().writeOpenGlMatrix(glMatrix);
	    glMultMatrixd( glMatrix );
#endif
      for_each(this, node, node->collisionModel, &ColourPickingVisitor::processCollisionModel);
      glPopMatrix();

      return RESULT_CONTINUE;
    }

    void ColourPickingVisitor::processCollisionModel(simulation::Node* /* node */ , core::CollisionModel* o)
    {
      using namespace core::objectmodel;
      o->drawColourPicking(method);
    }

  }
}
