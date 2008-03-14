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
#ifndef SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MECHANICALMAPPING_H
#define SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MECHANICALMAPPING_H

#include <sofa/core/Mapping.h>
#include <sofa/core/componentmodel/behavior/BaseMechanicalMapping.h>

namespace sofa
{

namespace core
{

namespace componentmodel
{

namespace behavior
{

/**
 *  \brief Specialized interface to convert a mechanical model of type In to
 *  another mechanical model of type Out.
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
template <class In, class Out>
class MechanicalMapping : public Mapping<In,Out>, public BaseMechanicalMapping
{
public:
    Data<bool> f_isMechanical;
    
    MechanicalMapping(In* from, Out* to);

    virtual ~MechanicalMapping();

    /// Apply the reverse mapping on force vectors.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ out += J^t in $
    ///
    /// This method must be reimplemented by all mappings.
    virtual void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in ) = 0;

    /// Apply the reverse mapping on constraint matrix.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ Out += J^t In $
    ///
    /// This method must be reimplemented by all mappings if they need to support constraints.
    virtual void applyJT( typename In::VecConst& /*out*/, const typename Out::VecConst& /*in*/ ) {}

    /// Get the source (upper) model.
    virtual BaseMechanicalState* getMechFrom();

    /// Get the destination (lower, mapped) model.
    virtual BaseMechanicalState* getMechTo();

    /// Return false if this mapping should only be used as a regular mapping instead of a mechanical mapping.
    virtual bool isMechanical();

    virtual void init();

    /// Propagate position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ x_out = J x_in $
    ///
    /// This method retrieves the x vectors and call the internal apply() method implemented by the component.
    virtual void propagateX();

    /// Propagate free-motion position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ xfree_out = J xfree_in $
    ///
    /// This method retrieves the xfree vectors and call the internal apply() method implemented by the component.
    virtual void propagateXfree();

    /// Propagate velocity from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ v_out = J v_in $
    ///
    /// This method retrieves the v vectors and call the internal apply() method implemented by the component.
    virtual void propagateV();

    /// Propagate displacement from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ dx_out = J dx_in $
    ///
    /// This method retrieves the dx vectors and call the internal applyJ() method implemented by the component.
    virtual void propagateDx();

    /// Accumulate forces from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ f_in += J^t f_out $
    ///
    /// This method retrieves the force vectors and call the internal applyJT() method implemented by the component.
    virtual void accumulateForce();

    /// Accumulate force deltas from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ df_in += J^t df_out $
    ///
    /// This method retrieves the force vectors and call the internal applyJT() method implemented by the component.
    virtual void accumulateDf();

    /// Accumulate constraint from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ C_in += J^t C_out $
    ///
    /// This method retrieves the constraint matrices and call the internal applyJT() method implemented by the component.
    virtual void accumulateConstraint();

};

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

#endif
