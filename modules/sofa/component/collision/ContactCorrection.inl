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
#ifndef SOFA_CORE_COMPONENTMODEL_COLLISION_CONTACTCORRECTION_INL
#define SOFA_CORE_COMPONENTMODEL_COLLISION_CONTACTCORRECTION_INL

#include "ContactCorrection.h"
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/component/mass/UniformMass.h>
#include <sofa/simulation/tree/MechanicalVisitor.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
ContactCorrection<DataTypes>::ContactCorrection(behavior::MechanicalState<DataTypes> *mm)
: mstate(mm)
{
}

template<class DataTypes>
ContactCorrection<DataTypes>::~ContactCorrection()
{
}

template<class DataTypes>
void ContactCorrection<DataTypes>::init()
{
    mstate = dynamic_cast< behavior::MechanicalState<DataTypes>* >(getContext()->getMechanicalState());
}

template<>
void ContactCorrection<defaulttype::Rigid3Types>::getCompliance(double**W)
{
	const VecConst& constraints = *mstate->getC();
	Deriv weighedNormal;
	Deriv InvM_wN;

	const sofa::defaulttype::Rigid3Mass* massValue;

	simulation::tree::GNode *node = dynamic_cast<simulation::tree::GNode *>(getContext());

	if (node != NULL)
	{
		core::componentmodel::behavior::BaseMass*_m = node->mass;
		component::mass::UniformMass<defaulttype::Rigid3Types, defaulttype::Rigid3Mass> *m = dynamic_cast<component::mass::UniformMass<defaulttype::Rigid3Types, defaulttype::Rigid3Mass>*> (_m);
		massValue = &( m->getMass());
	}
	else
	{
		massValue = new sofa::defaulttype::Rigid3Mass();
		printf("\n WARNING : node is not found => massValue could be false in getCompliance function");
	}

	unsigned int numConstraints = constraints.size();

	double dt = this->getContext()->getDt();

	for(unsigned int curRowConst = 0; curRowConst < numConstraints; curRowConst++)
	{
		int sizeCurRowConst = constraints[curRowConst].size();
		int indexCurRowConst = mstate->getConstraintId()[curRowConst];

		for(int i = 0; i < sizeCurRowConst; i++)
		{
			weighedNormal.getVCenter() = constraints[curRowConst][i].data.getVCenter(); // weighed normal
			weighedNormal.getVOrientation() = constraints[curRowConst][i].data.getVOrientation();

			InvM_wN = weighedNormal / (*massValue);
			InvM_wN *= dt*dt;
		
			int indexCurColConst;

			for(unsigned int curColConst = curRowConst; curColConst < numConstraints; curColConst++)
			{
				int sizeCurColConst = constraints[curColConst].size();
				indexCurColConst = mstate->getConstraintId()[curColConst];

				for(int j = 0; j < sizeCurColConst; j++)
				{
					W[indexCurRowConst][indexCurColConst] +=  constraints[curColConst][j].data * InvM_wN;
				}
			}

			for(unsigned int curColConst = curRowConst+1; curColConst < numConstraints; curColConst++)
			{
				indexCurColConst = mstate->getConstraintId()[curColConst];
				W[indexCurColConst][indexCurRowConst] = W[indexCurRowConst][indexCurColConst];
			}

		}
	}
}

template<>
void ContactCorrection<defaulttype::Rigid3Types>::applyContactForce(double *f)
{
	VecDeriv& force = *mstate->getExternalForces();
	const VecConst& constraints = *mstate->getC();
	Deriv weighedNormal;

	const sofa::defaulttype::Rigid3Mass* massValue;

	simulation::tree::GNode *node = dynamic_cast<simulation::tree::GNode *>(getContext());

	if (node != NULL)
	{
		core::componentmodel::behavior::BaseMass*_m = node->mass;
		component::mass::UniformMass<defaulttype::Rigid3Types, defaulttype::Rigid3Mass> *m = dynamic_cast<component::mass::UniformMass<defaulttype::Rigid3Types, defaulttype::Rigid3Mass>*> (_m);
		massValue = &( m->getMass());
	}
	else
	{
		massValue = new sofa::defaulttype::Rigid3Mass();
		printf("\n WARNING : node is not found => massValue could be false in getCompliance function");
	}


	double dt = this->getContext()->getDt();

	force.resize(0);
	force.resize(1);
	force[0] = Deriv();

	int numConstraints = constraints.size();

	for(int c1 = 0; c1 < numConstraints; c1++)
	{
		int indexC1 = mstate->getConstraintId()[c1];

		if (f[indexC1] != 0.0)
		{
			int sizeC1 = constraints[c1].size();
			for(int i = 0; i < sizeC1; i++)
			{	
				weighedNormal = constraints[c1][i].data; // weighted normal
				force[0].getVCenter() += weighedNormal.getVCenter() * f[indexC1];
				force[0].getVOrientation() += weighedNormal.getVOrientation() * f[indexC1];
			}
		}
	}


	VecDeriv& dx = *mstate->getDx();
	VecCoord& x = *mstate->getX();
	VecDeriv& v = *mstate->getV();
	VecDeriv& v_free = *mstate->getVfree();
	VecCoord& x_free = *mstate->getXfree();


//	mstate->setX(x_free);
//	mstate->setV(v_free);
	x[0]=x_free[0];
	v[0]=v_free[0];

	// Euler integration... will be done in the "integrator" as soon as it exists !
	dx.resize(v.size());
	dx[0] = force[0] / (*massValue);
	dx[0] *= dt;
	v[0] += dx[0];
	dx[0] *= dt;
	x[0] += dx[0];
//	simulation::tree::MechanicalPropagateAndAddDxVisitor(dx).execute(this->getContext());

}

template<>
void ContactCorrection<defaulttype::Vec1dTypes>::getCompliance(double**W)
{
	const VecConst& constraints = *mstate->getC();
	unsigned int numConstraints = constraints.size();

	for(unsigned int curRowConst = 0; curRowConst < numConstraints; curRowConst++)
	{
		int sizeCurRowConst = constraints[curRowConst].size();
		int indexCurRowConst = mstate->getConstraintId()[curRowConst];

		for(int i = 0; i < sizeCurRowConst; i++)
		{
			int indexCurColConst;

			for(unsigned int curColConst = curRowConst; curColConst < numConstraints; curColConst++)
			{
				int sizeCurColConst = constraints[curColConst].size();
				indexCurColConst = mstate->getConstraintId()[curColConst];

				for(int j = 0; j < sizeCurColConst; j++)
				{
					if (constraints[curRowConst][i].index == constraints[curColConst][j].index)
					{
						W[indexCurRowConst][indexCurColConst] += (1.0/10000.0) * constraints[curRowConst][i].data.x() * constraints[curColConst][j].data.x();
					}
				}
			}

			for(unsigned int curColConst = curRowConst+1; curColConst < numConstraints; curColConst++)
			{
				indexCurColConst = mstate->getConstraintId()[curColConst];
				W[indexCurColConst][indexCurRowConst] = W[indexCurRowConst][indexCurColConst];
			}

		}
	}

// debug : verifie qu'il n'y a pas de 0 sur la diagonale de W
	//printf("\n index : ");
	//for(unsigned int curRowConst = 0; curRowConst < numConstraints; curRowConst++)
	//{
	//	int indexCurRowConst = mstate->getConstraintId()[curRowConst];
	//	printf(" %d ",indexCurRowConst);
	//	if(abs(W[indexCurRowConst][indexCurRowConst]) < 0.000000001)
	//		printf("\n WARNING : there is a 0 on the diagonal of matrix W");

	//	if(abs(W[curRowConst][curRowConst]) <0.000000001)
	//		printf("\n stop");
	//}


}

template<>
void ContactCorrection<defaulttype::Vec1dTypes>::applyContactForce(double *f){

	VecDeriv& force = *mstate->getExternalForces();
	const VecConst& constraints = *mstate->getC();
	unsigned int numConstraints = constraints.size();

	force.resize((*mstate->getX()).size());

	for(unsigned int c1 = 0; c1 < numConstraints; c1++)
	{
		int indexC1 = mstate->getConstraintId()[c1];

		if (f[indexC1] != 0.0)
		{
			int sizeC1 = constraints[c1].size();
			for(int i = 0; i < sizeC1; i++)
			{	
				force[constraints[c1][i].index] += constraints[c1][i].data * f[indexC1];
			}
		}
	}

	VecDeriv& dx = *mstate->getDx();
	VecCoord& x = *mstate->getX();
	VecDeriv& v = *mstate->getV();
	VecDeriv& v_free = *mstate->getVfree();
	VecCoord& x_free = *mstate->getXfree();
	double dt = this->getContext()->getDt();


	// Euler integration... will be done in the "integrator" as soon as it exists !
	dx.resize(v.size());

	for (unsigned int i=0; i<dx.size(); i++)
	{
		x[i] = x_free[i];
		v[i] = v_free[i];
		dx[i] = force[i]/10000.0;
		x[i] += dx[i];
		v[i] += dx[i]/dt;
	}
}

template<class DataTypes>
void ContactCorrection<DataTypes>::resetContactForce()
{
	VecDeriv& force = *mstate->getExternalForces();
	for( unsigned i=0; i<force.size(); ++i )
		force[i] = Deriv();
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
