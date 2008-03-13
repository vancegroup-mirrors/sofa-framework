#ifndef SOFA_CORE_COMPONENTMODEL_BEHAVIOR_BASEMODEL_H
#define SOFA_CORE_COMPONENTMODEL_BEHAVIOR_BASEMODEL_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

namespace componentmodel
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
 *  \li \code VecDeriv \endcode : container of Deriv values with the same API as sofa::helper::vector *  \li \code SparseDeriv \endcode : index + Deriv value (entry of a sparse vector).
 *  \li \code SparseVecDeriv \endcode : sparse vector of Deriv values (defining coefficient of a constraint).
 *  \li \code VecConst \endcode : vector of constraints (i.e. of SparseVecDeriv).
 *
 *  \todo sofa::core::componentmodel::behavior::State is related to sofa::core::Mapping, and not to sofa::core::componentmodel::behavior::MechanicalMapping, so why is it in the same namespace ? Maybe we should put it in componentmodel namespace instead or directly in Core.
 *
 */
template<class TDataTypes>
class State : public virtual objectmodel::BaseObject
{
public:
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
    /// Index + Deriv value (entry of a sparse vector).
    typedef typename DataTypes::SparseDeriv SparseDeriv;
    /// Sparse vector of Deriv values (defining coefficient of a constraint).
    typedef typename DataTypes::SparseVecDeriv SparseVecDeriv;
    /// Vector of constraints (i.e. of SparseVecDeriv).
    typedef typename DataTypes::VecConst VecConst;

    virtual ~State() { }

    /// Return the current position vector (read-write access).
    virtual VecCoord* getX() = 0;
    /// Return the current velocity vector (read-write access).
    virtual VecDeriv* getV() = 0;

    /// Return the current position vector (read-only access).
    virtual const VecCoord* getX()  const = 0;
    /// Return the current velocity vector (read-only access).
    virtual const VecDeriv* getV()  const = 0;
};

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

#endif
