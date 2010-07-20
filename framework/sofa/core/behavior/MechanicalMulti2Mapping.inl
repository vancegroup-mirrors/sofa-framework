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
#ifndef SOFA_CORE_BEHAVIOR_MECHANICALMULTI2MAPPING_INL
#define SOFA_CORE_BEHAVIOR_MECHANICALMULTI2MAPPING_INL


#include <sofa/core/Multi2Mapping.h>
#include <sofa/core/Multi2Mapping.inl>
#include <sofa/core/behavior/MechanicalMulti2Mapping.h>
#include <algorithm>
#include <functional>

namespace sofa
{

namespace core
{

namespace behavior
{

template < class In1, class In2,class Out>
MechanicalMulti2Mapping<In1,In2,Out>::MechanicalMulti2Mapping()
: Multi2Mapping<In1,In2,Out>()
, f_isMechanical( initData( &f_isMechanical, true, "isMechanical", "set to false if this mapping should only be used as a regular mapping instead of a mechanical mapping" ) )
{
}
          
          template < class In1, class In2,class Out>
          bool MechanicalMulti2Mapping<In1,In2,Out>::isMechanical()
          {
              return this->f_isMechanical.getValue();
          }

          template < class In1, class In2,class Out>
          void MechanicalMulti2Mapping<In1,In2,Out>::setMechanical(bool b)
          {
            f_isMechanical.setValue(b);
          }


          template < class In1, class In2,class Out>
          helper::vector<BaseMechanicalState*> MechanicalMulti2Mapping<In1,In2,Out>::getMechFrom()
          {
            helper::vector<BaseMechanicalState*> mechFromVec;
            std::copy(this->fromModels1.begin(), this->fromModels1.end(), std::back_inserter(mechFromVec));
            std::copy(this->fromModels2.begin(), this->fromModels2.end(), std::back_inserter(mechFromVec));
            return mechFromVec;
          }

          template < class In1, class In2,class Out>
          helper::vector<BaseMechanicalState*> MechanicalMulti2Mapping<In1,In2,Out>::getMechTo()
          {
            helper::vector<BaseMechanicalState*> mechToVec;
            std::copy(this->toModels.begin(), this->toModels.end(), std::back_inserter(mechToVec));
            return mechToVec;
          }


         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::init()
         {
           	this->updateMapping();
            propagateXfree();
         }
 
         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::propagateX()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;
           const VecId &idCoord = VecId::position();
           helper::vector<typename Out::VecCoord*> vecOutPos;
           getVecOutCoord(idCoord, vecOutPos);
           helper::vector<const typename In1::VecCoord*> vecIn1Pos;
           getConstVecIn1Coord(idCoord, vecIn1Pos);
           helper::vector<const typename In2::VecCoord*> vecIn2Pos;
           getConstVecIn2Coord(idCoord, vecIn2Pos);

           apply ( vecOutPos, vecIn1Pos, vecIn2Pos);
         }

         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::propagateXfree()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;
           const VecId &idDeriv = VecId::freePosition();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In1::VecDeriv*> vecIn1Vel;
           getConstVecIn1Deriv(idDeriv, vecIn1Vel);
           helper::vector<const typename In2::VecDeriv*> vecIn2Vel;
           getConstVecIn2Deriv(idDeriv, vecIn2Vel);

           applyJ( vecOutVel, vecIn1Vel, vecIn2Vel);
         }

         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::propagateDx()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;
           const VecId &idDeriv = VecId::dx();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In1::VecDeriv*> vecIn1Vel;
           getConstVecIn1Deriv(idDeriv, vecIn1Vel);
           helper::vector<const typename In2::VecDeriv*> vecIn2Vel;
           getConstVecIn2Deriv(idDeriv, vecIn2Vel);

           applyJ( vecOutVel, vecIn1Vel, vecIn2Vel);
         }

         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::propagateV()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;
           const VecId &idDeriv = VecId::velocity();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In1::VecDeriv*> vecIn1Vel;
           getConstVecIn1Deriv(idDeriv, vecIn1Vel);
           helper::vector<const typename In2::VecDeriv*> vecIn2Vel;
           getConstVecIn2Deriv(idDeriv, vecIn2Vel);

           applyJ( vecOutVel, vecIn1Vel, vecIn2Vel);
         }


         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::propagateA()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;
           const VecId &v = VecId::velocity();
           const VecId &dx = VecId::dx();

           helper::vector<typename Out::VecDeriv*> vecOutDx;
           getVecOutDeriv(dx, vecOutDx);

           helper::vector<const typename In1::VecDeriv*> vecIn1V;
           getConstVecIn1Deriv(v, vecIn1V);
           helper::vector<const typename In2::VecDeriv*> vecIn2V;
           getConstVecIn2Deriv(v, vecIn2V);

           helper::vector<const typename In1::VecDeriv*> vecIn1Dx;
           getConstVecIn1Deriv(dx, vecIn1Dx);
           helper::vector<const typename In2::VecDeriv*> vecIn2Dx;
           getConstVecIn2Deriv(dx, vecIn2Dx);

           this->computeAccFromMapping( vecOutDx, vecIn1V, vecIn2V, vecIn1Dx, vecIn2Dx );
         }

         template <class T, class Container>
         struct GetForceVectorFunctor
         {
           GetForceVectorFunctor(Container &vector):v(vector){};
           void operator()(T* mstate)
           {
             v.push_back(mstate->getVecDeriv(mstate->getForceId().index));
           }

         protected:
           Container &v;
         };


         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::accumulateForce()
         {
            if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
             return;

            helper::vector<typename In1::VecDeriv*> vecOut1Force;
            GetForceVectorFunctor<In1, helper::vector<typename In1::VecDeriv*> > OutF1(vecOut1Force);
            std::for_each(this->fromModels1.begin(), this->fromModels1.end(), OutF1);


            helper::vector<typename In2::VecDeriv*> vecOut2Force;
            GetForceVectorFunctor<In2, helper::vector<typename In2::VecDeriv*> > OutF2(vecOut2Force);
            std::for_each(this->fromModels2.begin(), this->fromModels2.end(), OutF2);


            helper::vector<const typename Out::VecDeriv*> vecInForce;
            GetForceVectorFunctor<Out, helper::vector<const typename Out::VecDeriv*> > InF(vecInForce);
            std::for_each(this->toModels.begin(), this->toModels.end(), InF);

            this->applyJT(vecOut1Force, vecOut2Force,vecInForce);
         }

         template < class In1, class In2,class Out>
         void MechanicalMulti2Mapping<In1,In2,Out>::accumulateDf()
         {
           if( (this->fromModels1.empty() && this->fromModels2.empty() )|| this->toModels.empty() )
            return;

           helper::vector<typename In1::VecDeriv*> vecOut1Force;
           GetForceVectorFunctor<In1, helper::vector<typename In1::VecDeriv*> > OutF1(vecOut1Force);
           std::for_each(this->fromModels1.begin(), this->fromModels1.end(), OutF1);


           helper::vector<typename In2::VecDeriv*> vecOut2Force;
           GetForceVectorFunctor<In2, helper::vector<typename In2::VecDeriv*> > OutF2(vecOut2Force);
           std::for_each(this->fromModels2.begin(), this->fromModels2.end(), OutF2);


           helper::vector<const typename Out::VecDeriv*> vecInForce;
           GetForceVectorFunctor<Out, helper::vector<const typename Out::VecDeriv*> > InF(vecInForce);
           std::for_each(this->toModels.begin(), this->toModels.end(), InF);

           this->applyJT(vecOut1Force, vecOut2Force,vecInForce);
         }
      
} // namespace behavior

} // namespace core

} // namespace sofa

#endif
