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
#ifndef SOFA_COMPONENT_CONSTRAINT_BASEPROJECTIVELMCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINT_BASEPROJECTIVELMCONSTRAINT_H

#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/core/componentmodel/behavior/LMConstraint.h>
#include <sofa/component/topology/PointSubset.h>


namespace sofa
{

  namespace component
  {

    namespace constraint
    {

      using namespace sofa::core::componentmodel::topology;

      /** Utility Class: Implements buildJabocian and writeConstraintEquations. Constrain a set of particle to given positions/velocities/acceleration.
       *  To use it, derive your constraint from the BaseProjectiveLMConstraint, and implements:
       *     - getExpectedAcceleration/getExpected/getExpectedPosition
       *     - getXDirection/getYDirection/getZDirection
       *  --> Applications: Fix some particles in space, line/plane constraint (constraining only two, or one degree of freedom), Guided particle constraint ...
       */
      template <class DataTypes>
	class BaseProjectiveLMConstraint :  public core::componentmodel::behavior::LMConstraint<DataTypes,DataTypes>
	{
	public:
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

	public:
	BaseProjectiveLMConstraint( MechanicalState *dof):
          core::componentmodel::behavior::LMConstraint<DataTypes,DataTypes>(dof,dof),
            f_indices(core::objectmodel::Base::initData(&f_indices, "indices", "List of the index of particles to be constrained"))
	      {};
	BaseProjectiveLMConstraint():
          f_indices(core::objectmodel::Base::initData(&f_indices, "indices", "List of the index of particles to be constrained"))
              {}

	  virtual ~BaseProjectiveLMConstraint(){}; 
	  
	  virtual void init();


          //*********************************************************************
	  // -- LMConstraint interface
          virtual void buildJacobian();
	  virtual void writeConstraintEquations(ConstOrder order);

          //*********************************************************************
          // -- BaseProjectiveLMConstraint API
          // Expected values for a given particle: 
          virtual void getExpectedAcceleration(unsigned int /* index */, helper::vector< SReal >&/* expectedValue */){};
          virtual void getExpectedVelocity    (unsigned int /* index */, helper::vector< SReal >&/* expectedValue */){};
          virtual void getPositionCorrection  (unsigned int /* index */, helper::vector< SReal >&/* correction */ )  {};
          //*********************************************************************

          std::string getTemplateName() const
            {
              return templateName(this);
            }
          static std::string templateName(const BaseProjectiveLMConstraint<DataTypes>* = NULL)
          {
            return DataTypes::Name();
          }

          // Handle topological changes
          virtual void handleTopologyChange();
          
          bool useMask(){return true;}
	protected :
          
          bool usingACC, usingVEL, usingPOS;

          /// set of particles to be constrained
          Data<SetIndex> f_indices;
          helper::vector<Deriv> constraintDirection;

          // Array storing the indices of the lines inside the Jacobian Matrix
          helper::vector< SetIndexArray > idx;
          sofa::core::componentmodel::topology::BaseMeshTopology* topology;       

          
          void clearConstraints();
          void addConstraint(unsigned int index);
          void removeConstraint(unsigned int index);

          // Define TestNewPointFunction
          static bool FCTestNewPointFunction(int, void*, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >& );
          // Define RemovalFunction
          static void FCRemovalFunction ( int , void*);
	};

    } // namespace constraint

  } // namespace component

} // namespace sofa

#endif
