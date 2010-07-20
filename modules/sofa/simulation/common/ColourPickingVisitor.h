#ifndef SOFA_SIMULATION_COLOURPICKING_ACTION
#define SOFA_SIMULATION_COLOURPICKING_ACTION

#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Visitor.h>
#include <sofa/core/CollisionModel.h>

namespace sofa{
  namespace simulation{
  
    /* Launches the drawColourPicking() method of each CollisionModel */
    class SOFA_SIMULATION_COMMON_API ColourPickingVisitor : public Visitor
    {
      public:
        ColourPickingVisitor(core::CollisionModel::ColourCode Method):method(Method){};
    void processCollisionModel(simulation::Node* /*node*/, core::CollisionModel* /*o*/); 

    virtual Result processNodeTopDown(simulation::Node* node);

	    /// Return a category name for this action.
	    /// Only used for debugging / profiling purposes
	      virtual const char* getCategoryName() const { return "collision"; }
        virtual const char* getClassName() const { return "ColourPickingVisitor"; }
    private:
      core::CollisionModel::ColourCode method;
    };

   
  
  }
}



#endif // SOFA_SIMULATION_COLLISIONDRAW_ACTION
