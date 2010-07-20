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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_MAPPEDMODEL_H
#define SOFA_CORE_BEHAVIOR_MAPPEDMODEL_H

#include <sofa/core/behavior/State.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Component storing position and velocity vectors as computed by a Mapping.
 *
 *  This class define the interface of components used as destination of regular
 *  (non mechanical) mapping. VisualModel implementations often implements it.
 *
 *  The given DataTypes class should define the following internal types:
 *  \li \code Real \endcode : scalar values (float or double).
 *  \li \code Coord \endcode : position values.
 *  \li \code Deriv \endcode : derivative values (velocity).
 *  \li \code VecReal \endcode : container of scalar values with the same API as sofa::helper::vector.
 *  \li \code VecCoord \endcode : container of Coord values with the same API as sofa::helper::vector.
 *  \li \code VecDeriv \endcode : container of Deriv values with the same API as sofa::helper::vector.
 *
 *  \todo sofa::core::behavior::MappedModel is related to sofa::core::Mapping, and not to sofa::core::behavior::MechanicalMapping, so why is it in the same namespace ? Maybe we should put it in componentmodel namespace instead or directly in Core.
 *
 */
template<class TDataTypes>
class MappedModel : public State<TDataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MappedModel,TDataTypes), SOFA_TEMPLATE(State,TDataTypes));

    typedef TDataTypes DataTypes;
    /// Scalar values (float or double).
    typedef typename DataTypes::Real Real;
    /// Position values.
    typedef typename DataTypes::Coord Coord;
    /// Derivative values (velocity, forces, displacements).
    typedef typename DataTypes::Deriv Deriv;
    /// Container of Coord values with the same API as sofa::helper::vector.
    typedef typename DataTypes::VecCoord VecCoord;
    /// Container of Deriv values with the same API as sofa::helper::vector.
    typedef typename DataTypes::VecDeriv VecDeriv;

    virtual ~MappedModel() { }


    //Mapped Model does not store any rest position
    virtual VecCoord* getX0() { return NULL; };
    //Mapped Model does not store any reset position
    virtual VecCoord* getXReset() { return NULL; };
    //Mapped Model does not store any normal
    virtual VecCoord* getN() { return NULL; };

    //Mapped Model does not store any rest position
    virtual const VecCoord* getX0() const { return NULL; };
    //Mapped Model does not store any normal
    virtual const VecCoord* getN() const { return NULL; };
    //Mapped Model does not store any reset position
    virtual const VecCoord* getXReset() const { return NULL; };
};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
