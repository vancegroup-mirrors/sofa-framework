/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_FORCEFIELD_SURFACEPRESSUREFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_SURFACEPRESSUREFORCEFIELD_H


#include <sofa/core/componentmodel/behavior/ForceField.h>

namespace sofa { namespace core { namespace componentmodel { namespace topology { class BaseMeshTopology; } } } }


namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;


/**
 * @brief SurfacePressureForceField Class
 *
 * Implements a pressure force applied on a triangle or quad surface.
 * Each surfel receives a pressure in the direction of its normal.
 */
template<class DataTypes>
class SurfacePressureForceField : public core::componentmodel::behavior::ForceField<DataTypes>, public virtual core::objectmodel::BaseObject
{
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord    Coord   ;
    typedef typename DataTypes::Deriv    Deriv   ;
    typedef typename Coord::value_type   Real    ;

    enum State { INCREASE, DECREASE };

protected:

    sofa::core::componentmodel::topology::BaseMeshTopology* _topology;
    Data< Real > pressure; ///< Scalar pressure value applied on the surfaces.
    Data< Coord > min; ///< Lower bound of the pressured box.
    Data< Coord > max; ///< Upper bound of the pressured box.
    Data< bool > pulseMode; ///< In this mode, the pressure increases (or decreases) from 0 to pressure cyclicly.
    Data< Real > pressureSpeed; ///< Pressure variation in Pascal by second.

    State state; ///< In pulse mode, says wether pressure is increasing or decreasing.

public:

    SurfacePressureForceField();
    virtual ~SurfacePressureForceField();

    virtual void init();

    virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);
    virtual void addDForce (VecDeriv& /*df*/, const VecDeriv& /*dx*/) {}
    virtual double getPotentialEnergy(const VecCoord& x);

    void draw();

    void setPressure(const Real _pressure)
    {
        this->pressure = _pressure;
    }

protected:

    /**
     * @brief Triangle based surface pressure computation method.
     * Each vertice receives a force equal to 1/3 of the pressure applied on its belonging triangle.
     */
    virtual void addTriangleSurfacePressure(VecDeriv& /*f*/, const VecCoord& /*x*/, const VecDeriv& /*v*/, const Real& /*pressure*/);


    /**
     * @brief Quad based surface pressure computation method.
     * Each vertice receives a force equal to 1/4 of the pressure applied on its belonging quad.
     */
    virtual void addQuadSurfacePressure(VecDeriv& /*f*/, const VecCoord& /*x*/, const VecDeriv& /*v*/, const Real& /*pressure*/);


    /**
     * @brief Returns true if the x parameters belongs to the pressured box.
     */
    inline bool isInPressuredBox(const Coord& /*x*/) const;


    /**
     * @brief Returns next pressure value in pulse mode.
     * Pressure is computed according to the pressureSpeed attribute and the simulation time step.
     */
    const Real computePulseModePressure(void);
};


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif /* _SURFACEPRESSUREFORCEFIELD_H_ */
