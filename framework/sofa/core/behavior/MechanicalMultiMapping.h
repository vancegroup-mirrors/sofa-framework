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
#ifndef SOFA_CORE_BEHAVIOR_MECHANICALMULTIMAPPING_H
#define SOFA_CORE_BEHAVIOR_MECHANICALMULTIMAPPING_H

#include <sofa/core/MultiMapping.h>
#include <sofa/core/behavior/BaseMechanicalMapping.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

namespace sofa
{

namespace core
{

namespace behavior
{

template< class In, class Out >
class MechanicalMultiMapping : public MultiMapping<In,Out>, public BaseMechanicalMapping
{
public:
          SOFA_CLASS2(SOFA_TEMPLATE2(MechanicalMultiMapping,In,Out), SOFA_TEMPLATE2(MultiMapping,In,Out), BaseMechanicalMapping);

          Data<bool> f_isMechanical;

          MechanicalMultiMapping();

          virtual ~MechanicalMultiMapping() {};

          virtual helper::vector<BaseMechanicalState*> getMechFrom();
 
          virtual helper::vector<BaseMechanicalState*> getMechTo();

          virtual void init();

          virtual void applyJT( const helper::vector<typename In::VecDeriv*>& outDeriv , const helper::vector<const typename Out::VecDeriv*>& inDeriv ) = 0;

          virtual void applyJT( const helper::vector<typename In::VecConst*>& /*outConstraint*/ , const helper::vector<const typename Out::VecConst*>& /*inConstraint*/ ){};

          virtual void propagateX();

          virtual void propagateDx();

          virtual void propagateXfree();

          virtual void propagateV();

          virtual void propagateA();

          virtual void computeAccFromMapping( const helper::vector<typename Out::VecDeriv*>& /*outDx*/, const helper::vector<const typename In::VecDeriv*>& /*inV */, const helper::vector<const typename In::VecDeriv*>& /*inDx */ ){}


          virtual void accumulateForce();

          virtual void accumulateDf();

          /// Return false if this mapping should only be used as a regular mapping instead of a mechanical mapping.
          virtual bool isMechanical();

         /// Determine if this mapping should only be used as a regular mapping instead of a mechanical mapping.
         virtual void setMechanical(bool b);


          virtual std::string getTemplateName() const
          {
            return templateName(this);
          }

          static std::string templateName(const MechanicalMultiMapping<In, Out>* = NULL)
          {
            return std::string("MechanicalMultiMapping<")+In::DataTypes::Name() + std::string(",") + Out::DataTypes::Name() + std::string(">");
          }
        protected:
          bool getShow() const { return this->getContext()->getShowMechanicalMappings(); }

};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif 
