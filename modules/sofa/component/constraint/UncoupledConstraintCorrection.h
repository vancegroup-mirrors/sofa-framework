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
#ifndef SOFA_COMPONENT_CONSTRAINT_UNCOUPLEDCONSTRAINTCORRECTION_H
#define SOFA_COMPONENT_CONSTRAINT_UNCOUPLEDCONSTRAINTCORRECTION_H

#include <sofa/core/componentmodel/behavior/BaseConstraintCorrection.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>


namespace sofa
{

namespace component
{

namespace constraint
{

using namespace sofa::core;
using namespace sofa::core::componentmodel;
/**
 *  \brief Component computing contact forces within a simulated body using the compliance method.
 */
template<class TDataTypes>
class UncoupledConstraintCorrection : public componentmodel::behavior::BaseConstraintCorrection
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::VecConst VecConst;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;

    UncoupledConstraintCorrection(behavior::MechanicalState<DataTypes> *mm = NULL);

    virtual ~UncoupledConstraintCorrection();

    virtual void init();

    /// Retrieve the associated MechanicalState
    behavior::MechanicalState<DataTypes>* getMState() { return mstate; }

    virtual void getCompliance(double**W);

    virtual void applyContactForce(double *f);

    virtual void resetContactForce();

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, objectmodel::BaseContext* context, objectmodel::BaseObjectDescription* arg)
    {
        if (dynamic_cast<behavior::MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
            return false;
        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const UncoupledConstraintCorrection<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

protected:
    behavior::MechanicalState<DataTypes> *mstate;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
