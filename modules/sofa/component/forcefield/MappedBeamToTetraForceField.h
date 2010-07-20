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
//
// C++ Interface: MappedBeamToTetraForceField
//
// Description: 
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_COMPONENT_FORCEFIELD_MAPPEDBEAMTOTETRAFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_MAPPEDBEAMTOTETRAFORCEFIELD_H

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/component/forcefield/PlaneForceField.h>
#include <vector>

#include <sofa/component/typedef/Sofa_typedef.h>
#include <sofa/component/forcefield/BeamFEMForceField.h>
#include <sofa/component/mapping/BarycentricMapping.h>

#include <sofa/core/behavior/MechanicalState.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

/// A box of 6 PlaneForceField that can rotate
template<class DataTypes>
class MappedBeamToTetraForceField : public core::behavior::ForceField<DataTypes>
{
public:
  SOFA_CLASS(SOFA_TEMPLATE(MappedBeamToTetraForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

	typedef core::behavior::ForceField<DataTypes> Inherit;
	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
        typedef typename Coord::value_type Real;
        typedef typename defaulttype::Rigid3dTypes Rigid;
        typedef sofa::component::mapping::BarycentricMapping< MechanicalMapping< MechanicalState<Vec3dTypes>, MechanicalState<Rigid3dTypes> > >   BarycentricMapping3_VtoR;
protected:
        BeamFEMForceField<Rigid> *mappedBeamForceField;
        BarycentricMapping3_VtoR *mappingBeamTetra;
        core::behavior::MechanicalState<Vec3dTypes>* parenchymaMO;
        core::behavior::MechanicalState<Rigid>* beamMO;

        /*core::objectmodel::Data<Coord> _center;
	core::objectmodel::Data<Deriv> _size;
	core::objectmodel::Data<Real> _speed;

	core::objectmodel::Data<Real> _stiffness;
	core::objectmodel::Data<Real> _damping;

        defaulttype::Vec<6,PlaneForceFieldT*> _planes;*/

public:
        MappedBeamToTetraForceField(core::behavior::MechanicalState<DataTypes>* object=NULL, const std::string& /*name*/="")
        /*: core::behavior::ForceField<DataTypes>(object)
	, _center(initData(&_center, Coord(0,0,0), "center", "box center"))
	, _size(initData(&_size, Deriv(1,1,1), "size", "box size"))
	, _speed(initData(&_speed, (Real)0.001, "speed", "rotation speed"))
	, _stiffness(initData(&_stiffness, (Real)500.0, "stiffness", "penality force stiffness"))
        , _damping(initData(&_damping, (Real)5.0, "damping", "penality force damping"))*/
	{		
	}


        ~MappedBeamToTetraForceField()
	{
	}


        virtual void init();

	virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);
	
	virtual void addDForce (VecDeriv& df, const VecDeriv& dx, double kFactor, double bFactor);

        virtual double getPotentialEnergy(const VecCoord& x) const;

        void addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset);

	void draw();
        //bool addBBox(double* minBBox, double* maxBBox);
			
};

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
