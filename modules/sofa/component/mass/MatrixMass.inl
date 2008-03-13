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
#ifndef SOFA_COMPONENT_MASS_MATRIXMASS_INL
#define SOFA_COMPONENT_MASS_MATRIXMASS_INL

#include <sofa/component/mass/MatrixMass.h>

#include <sofa/helper/gl/template.h>


#include <sofa/component/topology/EdgeSetTopology.h>
#include <sofa/component/topology/TriangleSetTopology.h>
#include <sofa/component/topology/TetrahedronSetTopology.h>
#include <sofa/component/topology/GridTopology.h>
#include <sofa/component/topology/SparseGridTopology.h>

namespace sofa
{

namespace component
{

namespace mass
{

	using namespace	sofa::component::topology;
	using namespace core::componentmodel::topology;
using namespace sofa::defaulttype;
using namespace sofa::core::componentmodel::behavior;

using std::cerr;
using std::endl;





template <class DataTypes, class MassType>
MatrixMass<DataTypes, MassType>::~MatrixMass()
{
}


///////////////////////////////////////////


template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::clear()
{
    VecMass& masses = *f_mass.beginEdit();
    masses.clear();
    f_mass.endEdit();
}


template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::resize(int vsize)
{
    VecMass& masses = *f_mass.beginEdit();
    masses.resize(vsize);
    f_mass.endEdit();
}



///////////////////////////////////////////




// -- Mass interface
template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::addMDx(VecDeriv& res, const VecDeriv& dx, double factor)
{
	const VecMass &masses= *_usedMassMatrices;
    
	if (factor == 1.0)
	{
        for (unsigned int i=0;i<dx.size();i++)
        {
			res[i] += masses[i] * dx[i];
        }
	}
	else
		for (unsigned int i=0;i<dx.size();i++)
		{
			res[i] += masses[i] * dx[i] * factor;
		}
    
}

template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::accFromF(VecDeriv& , const VecDeriv& )
{
	cerr<<"void MatrixMass<DataTypes, MassType>::accFromF(VecDeriv& a, const VecDeriv& f) not yet implemented (need the matrix assembly and inversion)\n";
}

template <class DataTypes, class MassType>
double MatrixMass<DataTypes, MassType>::getKineticEnergy( const VecDeriv&  )
{
	cerr<<"void MatrixMass<DataTypes, MassType>::getKineticEnergy not yet implemented\n";
	return 0;
}

template <class DataTypes, class MassType>
double MatrixMass<DataTypes, MassType>::getPotentialEnergy( const VecCoord&  )
{
	cerr<<"void MatrixMass<DataTypes, MassType>::getPotentialEnergy not yet implemented\n";
    return 0;
}



template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::addForce(VecDeriv& f, const VecCoord& x, const VecDeriv& v)
{

	const VecMass &masses= *_usedMassMatrices;

    // gravity
	Vec3d g ( this->getContext()->getLocalGravity() );
	Deriv theGravity;
	DataTypes::set ( theGravity, g[0], g[1], g[2]);
    
    // velocity-based stuff
	core::objectmodel::BaseContext::SpatialVector vframe = this->getContext()->getVelocityInWorld();
	core::objectmodel::BaseContext::Vec3 aframe = this->getContext()->getVelocityBasedLinearAccelerationInWorld() ;

    // project back to local frame
	vframe = this->getContext()->getPositionInWorld() / vframe;
	aframe = this->getContext()->getPositionInWorld().backProjectVector( aframe );
    
    // add weight and inertia force
	for (unsigned int i=0;i<masses.size();i++) {
		f[i] += masses[i]*theGravity + core::componentmodel::behavior::inertiaForce(vframe,aframe,masses[i],x[i],v[i]);
	}
}









//////////////////////////////////



template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::init()
{
	Inherited::init();
	
	if (f_mass.getValue().empty())
	{
		clear();
		defaultDiagonalMatrices();
		_usingDefaultDiagonalMatrices=true;
	}
	
	assert( f_mass.getValue().size() == this->mstate->getX()->size() );
	
	if( this->_lumped.getValue() )
	{
		lumpMatrices();
		_usedMassMatrices = &_lumpedMasses;
	}
	else
	{
		_usedMassMatrices = &f_mass.getValue();
	}
}

template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::reinit()
{
	if( _usingDefaultDiagonalMatrices ) // in case where defaultValue is modified
	{
		clear();
		defaultDiagonalMatrices();
	}

	if( this->_lumped.getValue() ) // in case of _lumped is modified
	{
		lumpMatrices();
		_usedMassMatrices = &_lumpedMasses;
	}
	else
	{
		_usedMassMatrices = &f_mass.getValue();
	}
}

template <class DataTypes, class MassType>
MassType MatrixMass<DataTypes, MassType>::diagonalMass( const Real& m )
{
	MassType diagonalMatrixMass;
	diagonalMatrixMass.identity();
	return diagonalMatrixMass*m;
}

template <class DataTypes, class MassType>
MassType MatrixMass<DataTypes, MassType>::lump( const MassType& m )
{
	MassType lumpedM;
	lumpedM.fill(0);
	for (int i=0;i<m.getNbLines();i++)
	{
		lumpedM[i][i] = m.line(i).sum();
	}
	return lumpedM;
}

template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::lumpMatrices( )
{
	_lumpedMasses.clear();
	for (unsigned i=0;i<f_mass.getValue().size();++i)
	{
		_lumpedMasses.push_back( lump( f_mass.getValue()[i] ) );
	}
}

template <class DataTypes, class MassType>
void MatrixMass<DataTypes, MassType>::defaultDiagonalMatrices( )
{
	VecMass& masses = *f_mass.beginEdit();
	masses.resize(this->mstate->getX()->size());
	MassType diagonalMatrixMass = diagonalMass( _defaultValue.getValue() );
	for (unsigned i=0;i<masses.size();++i)
	{
		masses[i] = diagonalMatrixMass;
	}
	_usingDefaultDiagonalMatrices=true;
	f_mass.endEdit();
}

} // namespace mass

} // namespace component

} // namespace sofa

#endif
