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
#ifndef SOFA_SIMULATION_CONSTRAINTVISITOR_H
#define SOFA_SIMULATION_CONSTRAINTVISITOR_H


#include <sofa/simulation/common/Visitor.h>
#include <sofa/core/componentmodel/behavior/BaseMechanicalState.h>
#include <sofa/core/componentmodel/behavior/BaseLMConstraint.h>
#include <iostream>

namespace sofa
{

  namespace simulation
  {



    /** Visitor used to apply constraint
     */
    class SOFA_SIMULATION_COMMON_API ConstraintVisitor : public Visitor
    {
    public:
      typedef sofa::core::componentmodel::behavior::BaseMechanicalState::VecId VecId;
    ConstraintVisitor(VecId id, bool _propagateVelocityFromPosition):Id(id), propagateVelocityFromPosition(_propagateVelocityFromPosition), isEnd(RESULT_CONTINUE){}
    ConstraintVisitor():Id(VecId::position()), propagateVelocityFromPosition(false), isEnd(RESULT_CONTINUE){}

      virtual void processOdeSolver(simulation::Node* node, core::componentmodel::behavior::OdeSolver* obj);

      virtual Result processNodeTopDown(simulation::Node* node);
      //virtual void processNodeBottomUp(simulation::Node* node);

      /// Specify whether this action can be parallelized.
      virtual bool isThreadSafe() const { return true; }

      void setId(VecId id){Id=id;}
      void setPropagateVelocityFromPosition(bool b){propagateVelocityFromPosition=b;}
      /// Return a category name for this action.
      /// Only used for debugging / profiling purposes
      virtual const char* getCategoryName() const { return "constraint"; }
      virtual const char* getClassName() const { return "ConstraintVisitor"; }
	  virtual std::string getInfos() const { std::string name="[" + Id.getName() + "]"; return name; }
    protected:
      VecId Id;
      bool propagateVelocityFromPosition;
      Result isEnd;
    };


    class SOFA_SIMULATION_COMMON_API ConstraintErrorVisitor : public Visitor
    {
    public:
      typedef sofa::core::componentmodel::behavior::BaseMechanicalState::VecId VecId;
    ConstraintErrorVisitor():error(0){}

      virtual void processLMConstraint(simulation::Node* node, core::componentmodel::behavior::BaseLMConstraint* obj);
      virtual Result processNodeTopDown(simulation::Node* node);

      /// Specify whether this action can be parallelized.
      virtual bool isThreadSafe() const { return true; }

      /// Return a category name for this action.
      /// Only used for debugging / profiling purposes
      virtual const char* getCategoryName() const { return "constraint"; }
      virtual const char* getClassName() const { return "ConstraintErrorVisitor"; }
      double getError() const {return error;}
    protected:
      double error;
    };

  }
}

#endif
