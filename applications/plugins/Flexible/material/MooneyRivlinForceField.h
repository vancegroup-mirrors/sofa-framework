/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_MooneyRivlinFORCEFIELD_H
#define SOFA_MooneyRivlinFORCEFIELD_H

#include "../initFlexible.h"
#include "../material/BaseMaterialForceField.h"
#include "../material/MooneyRivlinMaterialBlock.inl"

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/common/AnimateEndEvent.h>

namespace sofa
{
namespace component
{
namespace forcefield
{

using helper::vector;

/** Apply MooneyRivlin's Law for isotropic homogeneous incompressible materials.
  * The energy is : C1 ( I1/ I3^1/3  - 3)  + C2 ( I2/ I3^2/3  - 3)
*/

template <class _DataTypes>
class SOFA_Flexible_API MooneyRivlinForceField : public BaseMaterialForceField<defaulttype::MooneyRivlinMaterialBlock<_DataTypes> >
{
public:
    typedef defaulttype::MooneyRivlinMaterialBlock<_DataTypes> BlockType;
    typedef BaseMaterialForceField<BlockType> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(MooneyRivlinForceField,_DataTypes),SOFA_TEMPLATE(BaseMaterialForceField, BlockType));

    typedef typename Inherit::Real Real;

    /** @name  Material parameters */
    //@{
    Data<vector<Real> > f_C1;
    Data<vector<Real> > f_C2;
    //@}

    virtual void reinit()
    {
        Real C1=0,C2=0;
        for(unsigned int i=0; i<this->material.size(); i++)
        {
            if(i<f_C1.getValue().size()) C1=f_C1.getValue()[i]; else if(f_C1.getValue().size()) C1=f_C1.getValue()[0];
            if(i<f_C2.getValue().size()) C2=f_C2.getValue()[i]; else if(f_C2.getValue().size()) C2=f_C2.getValue()[0];
            this->material[i].init( C1, C2 );
        }
        Inherit::reinit();
    }

    void handleEvent(sofa::core::objectmodel::Event *event)
    {
        if ( dynamic_cast<simulation::AnimateEndEvent*>(event))
        {
            if(f_C1.isDirty() || f_C2.isDirty()) reinit();
        }
    }


protected:
    MooneyRivlinForceField(core::behavior::MechanicalState<_DataTypes> *mm = NULL)
        : Inherit(mm)
        , f_C1(initData(&f_C1,vector<Real>((int)1,(Real)1000),"C1","weight of (~I1-3) term in energy"))
        , f_C2(initData(&f_C2,vector<Real>((int)1,(Real)1000),"C2","weight of (~I2-3) term in energy"))
//        , _viscosity(initData(&_viscosity,(Real)0,"viscosity","Viscosity (stress/strainRate)"))
    {
        this->f_listening.setValue(true);
    }

    virtual ~MooneyRivlinForceField()     {    }

};


}
}
}

#endif