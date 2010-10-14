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
#ifndef SOFA_CORE_BEHAVIOR_BASEMODEL_H
#define SOFA_CORE_BEHAVIOR_BASEMODEL_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Component storing position and velocity vectors.
 *
 *  This class define the interface of components used as source and
 *  destination of regular (non mechanical) mapping. It is then specialized as
 *  MechanicalState (storing other mechanical data) or MappedModel (if no
 *  mechanical data is used, such as for VisualModel).
 *
 *  The given DataTypes class should define the following internal types:
 *  \li \code Real \endcode : scalar values (float or double).
 *  \li \code Coord \endcode : position values.
 *  \li \code Deriv \endcode : derivative values (velocity).
 *  \li \code VecReal \endcode : container of scalar values with the same API as sofa::helper::vector.
 *  \li \code VecCoord \endcode : container of Coord values with the same API as sofa::helper::vector.
 *  \li \code VecDeriv \endcode : container of Deriv values with the same API as sofa::helper::vector
 *  \li \code SparseVecDeriv \endcode : sparse vector of Deriv values (defining coefficient of a constraint).
 *  \li \code VecConst \endcode : vector of constraints (i.e. of SparseVecDeriv).
 *
 *  \todo sofa::core::behavior::State is related to sofa::core::Mapping, and not to sofa::core::behavior::MechanicalMapping, so why is it in the same namespace ? Maybe we should put it in componentmodel namespace instead or directly in Core.
 *
 */
template<class TDataTypes>
class State : public virtual objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(State,TDataTypes), objectmodel::BaseObject);

    typedef TDataTypes DataTypes;
    /// Scalar values (float or double).
    typedef typename DataTypes::Real Real;
    /// Position values.
    typedef typename DataTypes::Coord Coord;
    /// Derivative values (velocity, forces, displacements).
    typedef typename DataTypes::Deriv Deriv;
    /// Container of scalar values with the same API as sofa::helper::vector.
    typedef typename DataTypes::VecReal VecReal;
    /// Container of Coord values with the same API as sofa::helper::vector.
    typedef typename DataTypes::VecCoord VecCoord;
    /// Container of Deriv values with the same API as sofa::helper::vector.
    typedef typename DataTypes::VecDeriv VecDeriv;
    ///// Sparse vector of Deriv values (defining coefficient of a constraint).
    //typedef typename DataTypes::SparseVecDeriv SparseVecDeriv;
    ///// Vector of constraints (i.e. of SparseVecDeriv).
    //typedef typename DataTypes::VecConst VecConst;
	/// Sparse matrix containing derivative values (constraints)
	typedef typename DataTypes::MatrixDeriv MatrixDeriv;

    virtual ~State() { }

    /// Resize all stored vector
    virtual void resize(int vsize) = 0;

    /// Return the current position vector (read-write access).
    virtual VecCoord* getX() = 0;
    /// Return the current velocity vector (read-write access).
    virtual VecDeriv* getV() = 0;
    /// Return the current rest position vector (read-write access)
    /// (return NULL if the state does not store rest position .
    virtual VecCoord* getX0() = 0;
    /// Return the current reset position vector (read-write access)
    /// (return NULL if the state does not store rest position .
    virtual VecCoord* getXReset() = 0;
    /// Return the current velocity vector (read-write access).
    /// (return NULL if the state does not store normal .
    virtual VecCoord* getN() = 0;

    /// Return the current position vector (read-only access).
    virtual const VecCoord* getX()  const = 0;
    /// Return the current velocity vector (read-only access).
    virtual const VecDeriv* getV()  const = 0;
    /// Return the current rest position vector (read-only access)
    /// (return NULL if the state does not store rest position .
    virtual const VecCoord* getX0() const = 0;
    /// Return the current reset position vector (read-write access)
    /// (return NULL if the state does not store rest position .
    virtual const VecCoord* getXReset() const = 0;
    /// Return the current velocity vector (read-only access).
    /// (return NULL if the state does not store normal .
    virtual const VecCoord* getN() const = 0;

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const State<DataTypes>* = NULL)
    {
        return TDataTypes::Name();
    }

    //static std::string Name(const State<DataTypes>* = NULL)
    //{
    //  return std::string("State");
    //}
};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
