/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINT_DOFBLOCKERLMCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINT_DOFBLOCKERLMCONSTRAINT_H

#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/core/componentmodel/behavior/LMConstraint.h>
#include <sofa/component/topology/PointSubset.h>
#include <sofa/component/linearsolver/LagrangeMultiplierComputation.h>
#include <sofa/simulation/common/Node.h>


namespace sofa
{

  namespace component
  {

    namespace constraint
    {

      using namespace sofa::core::componentmodel::topology;
      /// This class can be overridden if needed for additionnal storage within template specializations.
      template <class DataTypes>
    class DOFBlockerLMConstraintInternalData
	{
	};




      /** Keep two particules at an initial distance
       */
      template <class DataTypes>
    class DOFBlockerLMConstraint :  public core::componentmodel::behavior::LMConstraint<DataTypes,DataTypes>
	{
	public:
        SOFA_CLASS(SOFA_TEMPLATE(DOFBlockerLMConstraint,DataTypes),SOFA_TEMPLATE2(sofa::core::componentmodel::behavior::LMConstraint, DataTypes, DataTypes));

	  typedef typename DataTypes::VecCoord VecCoord;
	  typedef typename DataTypes::Coord Coord;
	  typedef typename DataTypes::VecDeriv VecDeriv;
	  typedef typename DataTypes::Deriv Deriv;
	  typedef typename DataTypes::SparseVecDeriv SparseVecDeriv;
	  typedef typename core::componentmodel::behavior::MechanicalState<DataTypes> MechanicalState;


    typedef sofa::component::topology::PointSubset SetIndex;
    typedef helper::vector<unsigned int> SetIndexArray;

	  typedef typename core::componentmodel::behavior::BaseMechanicalState::VecId VecId;
    typedef core::componentmodel::behavior::BaseLMConstraint::ConstOrder ConstOrder;

    typedef linearsolver::LagrangeMultiplierComputation::VectorEigen  VectorEigen;

	protected:
      DOFBlockerLMConstraintInternalData<DataTypes> data;
      friend class DOFBlockerLMConstraintInternalData<DataTypes>;
	
	public:
          DOFBlockerLMConstraint( MechanicalState *dof):
                  core::componentmodel::behavior::LMConstraint<DataTypes,DataTypes>(dof,dof),
                  BlockedAxis(core::objectmodel::Base::initData(&BlockedAxis, "rotationAxis", "List of rotation axis to constrain")),
                  factorAxis(core::objectmodel::Base::initData(&factorAxis, "factorAxis", "Factor to apply in order to block only a certain amount of rotation along the axis")),
                  f_indices(core::objectmodel::Base::initData(&f_indices, "indices", "List of the index of particles to be fixed")),
                  showSizeAxis(core::objectmodel::Base::initData(&showSizeAxis,(SReal)1.0,"showSizeAxis","size of the vector used to display the constrained axis") )
          { };
          DOFBlockerLMConstraint():
                  BlockedAxis(core::objectmodel::Base::initData(&BlockedAxis, "rotationAxis", "List of rotation axis to constrain")),
                  factorAxis(core::objectmodel::Base::initData(&factorAxis, "factorAxis", "Factor to apply in order to block only a certain amount of rotation along the axis")),
                  f_indices(core::objectmodel::Base::initData(&f_indices, "indices", "List of the index of particles to be fixed")),
                  showSizeAxis(core::objectmodel::Base::initData(&showSizeAxis,(SReal)1.0,"showSizeAxis","size of the vector used to display the constrained axis") )
          { };

      ~DOFBlockerLMConstraint(){};
	  
          void clearConstraints();
          void addConstraint(unsigned int index);
          void removeConstraint(unsigned int index);

          // Handle topological changes
          virtual void handleTopologyChange();
          
	  void init();
          void draw();
          void resetConstraint();

	  // -- LMConstraint interface
          void buildJacobian(unsigned int &constraintId);
	  void writeConstraintEquations(ConstOrder order);

    void LagrangeMultiplierEvaluation(const SReal* Wptr, SReal* cptr, SReal* LambdaInitptr,
                                      core::componentmodel::behavior::BaseLMConstraint::ConstraintGroup * group)
    {
      const unsigned int numConstraintToProcess=group->getNumConstraint();
      const VectorEigen &Lambda=linearsolver::LagrangeMultiplierComputation::ComputeLagrangeMultiplier(Wptr,cptr,LambdaInitptr,numConstraintToProcess);
      Eigen::Map<VectorEigen> LambdaInit(LambdaInitptr, numConstraintToProcess);
      LambdaInit = Lambda;
    }




          std::string getTemplateName() const
            {
              return templateName(this);
            }
          static std::string templateName(const DOFBlockerLMConstraint<DataTypes>* = NULL)
          {
            return DataTypes::Name();
          }


          bool isCorrectionComputedWithSimulatedDOF()
          {
              simulation::Node* node=(simulation::Node*) this->constrainedObject1->getContext();
              if (node->mechanicalMapping.empty()) return true;
              else return false;
          }
          bool useMask(){return true;}

          Data<helper::vector<Deriv> > BlockedAxis;
          Data<helper::vector<SReal> > factorAxis;
          Data<SetIndex> f_indices;

	protected :
          helper::vector<SetIndexArray> idxEquations;
          Data<SReal> showSizeAxis;


          sofa::core::componentmodel::topology::BaseMeshTopology* topology;
        

          // Define TestNewPointFunction
          static bool FCTestNewPointFunction(int, void*, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >& );

          // Define RemovalFunction
          static void FCRemovalFunction ( int , void*);

	};

    } // namespace constraint

  } // namespace component

} // namespace sofa

#endif
