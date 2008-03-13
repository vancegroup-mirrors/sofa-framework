/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_CORE_COMPONENTMODEL_BEHAVIOR_BASEMECHANICALMAPPING_H
#define SOFA_CORE_COMPONENTMODEL_BEHAVIOR_BASEMECHANICALMAPPING_H

#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/BehaviorModel.h>

namespace sofa
{

namespace core
{

namespace componentmodel
{

namespace behavior
{

class BaseMechanicalState;

/**
 *  \brief An interface to convert a mechanical model to another mechanical model
 *
 *  A MechanicalMapping is a mapping between two MechanicalState.
 *  Position, velocity and displacement are propagated from the source (upper)
 *  model to the destination (lower, or mapped) model, while forces and
 *  constraints are accumulated back from the destination model to the source
 *  model.
 *
 *  If the mapping is linear, then it can be represented by a matrix J, and
 *  most computations correspond to multiply this matrix or its transpose to
 *  the given vectors.
 */
class BaseMechanicalMapping : public virtual objectmodel::BaseObject
{
public:
    virtual ~BaseMechanicalMapping() { }

    /// Get the source (upper) model.
    virtual BaseMechanicalState* getMechFrom() = 0;
    /// Get the destination (lower, mapped) model.
    virtual BaseMechanicalState* getMechTo() = 0;

    /// Propagate position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ x_out = J x_in $
    virtual void propagateX() = 0;

    /// Propagate free-motion position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ xfree_out = J xfree_in $
    virtual void propagateXfree() = 0;

    /// Propagate velocity from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ v_out = J v_in $
    virtual void propagateV() = 0;

    /// Propagate displacement from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ dx_out = J dx_in $
    virtual void propagateDx() {}

    /// Accumulate forces from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ f_in += J^t f_out $
    virtual void accumulateForce() {}

    /// Accumulate force deltas from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ df_in += J^t df_out $
    virtual void accumulateDf() {}

    /// Accumulate constraint from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ C_in += J^t C_out $
    virtual void accumulateConstraint() {}

	/// Disable the mapping to get the original coordinates of the mapped model.
	///
	/// It is for instance used in RigidMapping to get the local coordinates of the object.
	virtual void disable() {}
};

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

#endif
