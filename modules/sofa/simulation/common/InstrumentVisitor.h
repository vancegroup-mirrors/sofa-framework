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
#ifndef SOFA_SIMULATION_INSTRUMENTACTION_H
#define SOFA_SIMULATION_INSTRUMENTACTION_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/simulation/common/Visitor.h>
#include <sofa/core/ExecParams.h>
namespace sofa
{

namespace simulation
{

class SOFA_SIMULATION_COMMON_API InstrumentVisitor : public Visitor
{

public:
	InstrumentVisitor(const core::ExecParams* params) : Visitor(params) {}

	virtual Result processNodeTopDown(simulation::Node* node);

	/// Specify whether this action can be parallelized.
	virtual bool isThreadSafe() const { return true; }

	/// Return a category name for this action.
	/// Only used for debugging / profiling purposes
	virtual const char* getCategoryName() const { return "instrument"; }
        virtual const char* getClassName() const { return "InstrumentVisitor"; }

	sofa::helper::vector< simulation::Node * > &getInstruments(){return instrumentNodes;}
 protected:
	sofa::helper::vector< simulation::Node * > instrumentNodes;
};

} // namespace simulation

} // namespace sofa

#endif
