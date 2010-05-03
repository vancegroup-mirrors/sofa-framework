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
#ifndef SOFA_COMPONENT_CONSTRAINT_POSITIONLMCONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINT_POSITIONLMCONSTRAINT_INL

#include <sofa/component/constraint/BaseProjectiveLMConstraint.h>
#include <sofa/simulation/common/Simulation.h>


namespace sofa
{

  namespace component
  {

    namespace constraint
    {

      using namespace sofa::helper;

      template <class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::init()
      {
        core::componentmodel::behavior::LMConstraint<DataTypes,DataTypes>::init();

	topology = this->getContext()->getMeshTopology();

	// Initialize functions and parameters
        topology::PointSubset my_subset = f_indices.getValue();

	my_subset.setTestFunction(FCTestNewPointFunction);
	my_subset.setRemovalFunction(FCRemovalFunction);

	my_subset.setTestParameter( (void *) this );
	my_subset.setRemovalParameter( (void *) this );
      }

      // Handle topological changes
      template <class DataTypes> void BaseProjectiveLMConstraint<DataTypes>::handleTopologyChange()
      {
	std::list<const TopologyChange *>::const_iterator itBegin=topology->firstChange();
	std::list<const TopologyChange *>::const_iterator itEnd =topology->lastChange();
        
	f_indices.beginEdit()->handleTopologyEvents(itBegin,itEnd,this->constrainedObject1->getSize());        
      }


      template<class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::buildJacobian()
      {
        idx.clear();
        idx.resize(constraintDirection.size());
        const SetIndexArray &indices = f_indices.getValue().getArray();

        for (SetIndexArray::const_iterator it = indices.begin(); it != indices.end(); ++it)
          {
            const unsigned int index=*it;

            for (unsigned int i=0;i<constraintDirection.size();++i)
              {
                //Constraint degree of freedom along X direction
                SparseVecDeriv V; V.add(index,constraintDirection[i]); 
                idx[i].push_back(registerEquationInJ1(V));
              }
            this->constrainedObject1->forceMask.insertEntry(index);
          }
      }


      template<class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::writeConstraintEquations(ConstOrder Order)
      {        

        switch(Order)
          {
          case core::componentmodel::behavior::BaseLMConstraint::ACC :
            {
              if (!usingACC) return;
              break;
            }
          case core::componentmodel::behavior::BaseLMConstraint::VEL :
            {
              if (!usingVEL) return;
              break;
            }
          case core::componentmodel::behavior::BaseLMConstraint::POS :
            {
              if (!usingPOS) return;
              break;
            }
          }


        const SetIndexArray & indices = f_indices.getValue().getArray();
        

        unsigned int counter=0;
        for (SetIndexArray::const_iterator it = indices.begin(); it != indices.end(); ++it,++counter)
          {
            const unsigned int index = *it;

            core::componentmodel::behavior::BaseLMConstraint::ConstraintGroup *constraint = this->addGroupConstraint(Order);
            helper::vector< SReal > correction;
            correction.resize(constraintDirection.size());

            switch(Order)
              {
              case core::componentmodel::behavior::BaseLMConstraint::ACC :
                {
                  helper::vector< SReal > expectedAcc(constraintDirection.size());
                  getExpectedAcceleration(index,expectedAcc);
                 
                  for (unsigned int i=0;i<constraintDirection.size();++i)
                    {
                      unsigned int lineNum=idx[i][counter];
                      SReal correction = expectedAcc[i]-this->constrainedObject1->getConstraintJacobianTimesVecDeriv(lineNum,core::componentmodel::behavior::BaseMechanicalState::VecId::dx());
                      constraint->addConstraint( lineNum, correction, core::componentmodel::behavior::BaseLMConstraint::BILATERAL);
                    }

                  break;
                }
              case core::componentmodel::behavior::BaseLMConstraint::VEL :
                {
                  helper::vector< SReal > expectedVel(constraintDirection.size());
                  getExpectedVelocity(index,expectedVel);
                 
                  for (unsigned int i=0;i<constraintDirection.size();++i)
                    {
                      unsigned int lineNum=idx[i][counter];
                      SReal correction = expectedVel[i]-this->constrainedObject1->getConstraintJacobianTimesVecDeriv(lineNum,core::componentmodel::behavior::BaseMechanicalState::VecId::velocity());
                      constraint->addConstraint( lineNum, correction, core::componentmodel::behavior::BaseLMConstraint::BILATERAL);
                    }

                  break;
                }
              case core::componentmodel::behavior::BaseLMConstraint::POS :
                {                 
                  helper::vector< SReal > correctionPos(constraintDirection.size());
                  getPositionCorrection(index, correctionPos);

                  for (unsigned int i=0;i<constraintDirection.size();++i)
                    {
                      unsigned int lineNum=idx[i][counter];
                      constraint->addConstraint( lineNum, correctionPos[i], core::componentmodel::behavior::BaseLMConstraint::BILATERAL);
                    }
                  break;
                }
              };
            
            
          }
      }


      // Define TestNewPointFunction
      template< class DataTypes>
      bool BaseProjectiveLMConstraint<DataTypes>::FCTestNewPointFunction(int /*nbPoints*/, void* param, const sofa::helper::vector< unsigned int > &, const sofa::helper::vector< double >& )
      {
	BaseProjectiveLMConstraint<DataTypes> *fc= (BaseProjectiveLMConstraint<DataTypes> *)param;
	if (fc) {
          return true;
	}else{
          return false;
	}
      }

      // Define RemovalFunction
      template< class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::FCRemovalFunction(int pointIndex, void* param)
      {
	BaseProjectiveLMConstraint<DataTypes> *fc= (BaseProjectiveLMConstraint<DataTypes> *)param;
	if (fc) {
          fc->removeConstraint((unsigned int) pointIndex);
	}
	return;
      }

      template <class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::clearConstraints()
      {
        f_indices.beginEdit()->clear();
        f_indices.endEdit();
      }

      template <class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::addConstraint(unsigned int index)
      {
        f_indices.beginEdit()->push_back(index);
        f_indices.endEdit();
      }

      template <class DataTypes>
      void BaseProjectiveLMConstraint<DataTypes>::removeConstraint(unsigned int index)
      {
        removeValue(*f_indices.beginEdit(),index);
        f_indices.endEdit();
      }



    } // namespace constraint

  } // namespace component

} // namespace sofa

#endif


