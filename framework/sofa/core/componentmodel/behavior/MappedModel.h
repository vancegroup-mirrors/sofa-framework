#ifndef SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MAPPEDMODEL_H
#define SOFA_CORE_COMPONENTMODEL_BEHAVIOR_MAPPEDMODEL_H

#include <sofa/core/componentmodel/behavior/State.h>

namespace sofa
{

namespace core
{

namespace componentmodel
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
 *  \todo sofa::core::componentmodel::behavior::MappedModel is related to sofa::core::Mapping, and not to sofa::core::componentmodel::behavior::MechanicalMapping, so why is it in the same namespace ? Maybe we should put it in componentmodel namespace instead or directly in Core.
 *
 */
template<class TDataTypes>
class MappedModel : public State<TDataTypes>
{
public:
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
};

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa

#endif
