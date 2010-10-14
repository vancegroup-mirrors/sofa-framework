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
#ifndef SOFA_CORE_BEHAVIOR_MECHANICALMULTIMAPPING_INL
#define SOFA_CORE_BEHAVIOR_MECHANICALMULTIMAPPING_INL


#include <sofa/core/MultiMapping.h>
#include <sofa/core/MultiMapping.inl>
#include <sofa/core/behavior/MechanicalMultiMapping.h>
#include <algorithm>
#include <functional>

namespace sofa
{

namespace core
{

namespace behavior
{

template <class In, class Out>
MechanicalMultiMapping<In,Out>::MechanicalMultiMapping()
: MultiMapping<In,Out>()
, f_isMechanical( initData( &f_isMechanical, true, "isMechanical", "set to false if this mapping should only be used as a regular mapping instead of a mechanical mapping" ) )
{
}

          template <class In, class Out>
          bool MechanicalMultiMapping<In,Out>::isMechanical()
          {
              return this->f_isMechanical.getValue();
          }

          template <class In, class Out>
          void MechanicalMultiMapping<In,Out>::setMechanical(bool b)
          {
            f_isMechanical.setValue(b);
          }

          template < class In, class Out >
          helper::vector<BaseMechanicalState*> MechanicalMultiMapping<In,Out>::getMechFrom()
          {
            helper::vector<BaseMechanicalState*> mechFromVec;
            std::copy(this->fromModels.begin(), this->fromModels.end(), std::back_inserter(mechFromVec));
            return mechFromVec;
          }

          template < class In, class Out >
          helper::vector<BaseMechanicalState*> MechanicalMultiMapping<In,Out>::getMechTo()
          {
            helper::vector<BaseMechanicalState*> mechToVec;
            std::copy(this->toModels.begin(), this->toModels.end(), std::back_inserter(mechToVec));
            return mechToVec;
          }


         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::init()
         {
            this->updateMapping();
            propagateXfree();
         }

#ifndef SOFA_SMP

         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateX()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
            return;

           }

           const VecId &idCoord = VecId::position();
           helper::vector<typename Out::VecCoord*> vecOutPos;
           getVecOutCoord(idCoord, vecOutPos);
           helper::vector<const typename In::VecCoord*> vecInPos;
           getConstVecInCoord(idCoord, vecInPos);

           apply ( vecOutPos, vecInPos);
         }

         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateXfree()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
            return;
           }
           const VecId &idDeriv = VecId::freePosition();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In::VecDeriv*> vecInVel;
           getConstVecInDeriv(idDeriv, vecInVel);

           applyJ( vecOutVel, vecInVel);
         }

         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateDx()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
             return;
           }
           const VecId &idDeriv = VecId::dx();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In::VecDeriv*> vecInVel;
           getConstVecInDeriv(idDeriv, vecInVel);

           applyJ( vecOutVel, vecInVel);
         }

         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateV()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
             return;
           }
           const VecId &idDeriv = VecId::velocity();
           helper::vector<typename Out::VecDeriv*> vecOutVel;
           getVecOutDeriv(idDeriv, vecOutVel);
           helper::vector<const typename In::VecDeriv*> vecInVel;
           getConstVecInDeriv(idDeriv, vecInVel);

           applyJ( vecOutVel, vecInVel);
         }


         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateA()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
             return;
           }
           const VecId &v = VecId::velocity();
           const VecId &dx = VecId::dx();

           helper::vector<typename Out::VecDeriv*> vecOutDx;
           getVecOutDeriv(dx, vecOutDx);
           helper::vector<const typename In::VecDeriv*> vecInV;
           getConstVecInDeriv(v, vecInV);
           helper::vector<const typename In::VecDeriv*> vecInDx;
           getConstVecInDeriv(dx, vecInDx);

           this->computeAccFromMapping( vecOutDx, vecInV, vecInDx );
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


         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::accumulateForce()
         {
            if( this->fromModels.empty() || this->toModels.empty() ){
             return;
           }

            helper::vector<typename In::VecDeriv*> vecOutForce;
            GetForceVectorFunctor<In, helper::vector<typename In::VecDeriv*> > OutF(vecOutForce);
            std::for_each(this->fromModels.begin(), this->fromModels.end(), OutF);


            helper::vector<const typename Out::VecDeriv*> vecInForce;
            GetForceVectorFunctor<Out, helper::vector<const typename Out::VecDeriv*> > InF(vecInForce);
            std::for_each(this->toModels.begin(), this->toModels.end(), InF);

            this->applyJT(vecOutForce, vecInForce);
         }

         template < class In, class Out>
         void MechanicalMultiMapping<In,Out>::accumulateDf()
         {
           if( this->fromModels.empty() || this->toModels.empty() ){
             return;
           }

           helper::vector<typename In::VecDeriv*> vecOutForce;
           GetForceVectorFunctor<In, helper::vector<typename In::VecDeriv*> > OutF(vecOutForce);
           std::for_each(this->fromModels.begin(), this->fromModels.end(), OutF);

           helper::vector<const typename Out::VecDeriv*> vecInForce;
           GetForceVectorFunctor<Out, helper::vector<const typename Out::VecDeriv*> > InF(vecInForce);
           std::for_each(this->toModels.begin(), this->toModels.end(), InF);

           this->applyJT(vecOutForce, vecInForce);
         }

#else

         using sofa::core::ParallelMultiMappingApply;
         using sofa::core::ParallelMultiMappingApplyJ;

         using sofa::core::ParallelMultiMappingApply3;
         using sofa::core::ParallelMultiMappingApplyJ3;

                  template <class T>
                  struct ParallelMultiMappingApplyJT3
                  {
                    void operator()(void *m, Shared_rw< typename T::In::VecDeriv> in1, Shared_rw<typename T::In::VecDeriv> in2,Shared_r<typename T::Out::VecDeriv> out){
                        out.read();
                        in1.access();
                        in2.access();
                        ((T *)m)->applyJT(((T *)m)->VecInForce, ((T *)m)->VecOutForce);
                    }
                  };
                  template <class T>
                  struct ParallelMultiMappingApplyJTCPU3
                  {
                    void operator()(void *m, Shared_rw< typename T::In::VecDeriv> in1, Shared_rw<typename T::In::VecDeriv> in2,Shared_r<typename T::Out::VecDeriv> out){
                        out.read();
                        in1.access();
                        in2.access();
                        ((T *)m)->applyJT(((T *)m)->VecInForce, ((T *)m)->VecOutForce);
                    }
                  };

                  template<class T>
                  struct ParallelComputeAccFromMultiMapping3
                  {
                    void operator()(void *m, Shared_rw<typename T::Out::VecDeriv> acc_out, Shared_r<typename T::In::VecDeriv> v_in1, Shared_r<typename T::In::VecDeriv> v_in2,Shared_r<typename T::In::VecDeriv> acc_in1,Shared_r<typename T::In::VecDeriv> acc_in2)
                    {
                        acc_out.access();
                        v_in1.read();
                        v_in2.read();
                        acc_in1.read();
                        acc_in2.read();

                      ((T *)m)->computeAccFromMapping(((T *)m)->VecOutDx2, ((T *)m)->VecInVel2, ((T *)m)->VecInDx2);
                    }
                  };


                  template <class In, class Out>
                  struct ParallelComputeAccFromMultiMapping3< MechanicalMultiMapping<In, Out> >
                  {
                          void operator()(MechanicalMultiMapping<In,Out> *m, Shared_rw<typename Out::VecDeriv> acc_out, Shared_r<typename In::VecDeriv> v_in1, Shared_r<typename In::VecDeriv> v_in2,Shared_r<typename In::VecDeriv> acc_in1,Shared_r<typename In::VecDeriv> acc_in2)
                          {
                              acc_out.access();
                              v_in1.read();
                              v_in2.read();
                              acc_in1.read();
                              acc_in2.read();
                              m->computeAccFromMapping(m->VecOutDx2, m->VecInVel2, m->VecInDx2);
                          }
                  };

                  template <class T>
                  struct ParallelMultiMappingApplyJT2
                  {
                    void operator()(void *m, Shared_rw< typename T::In::VecDeriv> in1, Shared_rw<typename T::In::VecDeriv> in2,Shared_r<typename T::Out::VecDeriv> out){
                        out.read();
                        in1.access();
                        in2.access();
                        ((T *)m)->applyJT(((T *)m)->VecInForce2, ((T *)m)->VecOutForce2);
                    }
                  };
                  template <class T>
                  struct ParallelMultiMappingApplyJTCPU2
                  {
                    void operator()(void *m, Shared_rw< typename T::In::VecDeriv> in1, Shared_rw<typename T::In::VecDeriv> in2,Shared_r<typename T::Out::VecDeriv> out){
                        out.read();
                        in1.access();
                        in2.access();
                        ((T *)m)->applyJT(((T *)m)->VecInForce2, ((T *)m)->VecOutForce2);
                    }
                  };

         template <class T>
         struct ParallelMultiMappingApplyJT
         {
           void operator()(void *m, Shared_rw< defaulttype::SharedVector<typename T::In::VecDeriv*> > out, Shared_r<defaulttype::SharedVector<const typename T::Out::VecDeriv* > > in){
             ((T *)m)->applyJT(out.access(), in.read());
           }
         };
         template <class T>
         struct ParallelMultiMappingApplyJTCPU
         {
           void operator()(void *m, Shared_rw< defaulttype::SharedVector<typename T::In::VecDeriv*> > out, Shared_r<defaulttype::SharedVector<const typename T::Out::VecDeriv* > > in){
             ((T *)m)->applyJT(out.access(), in.read());
           }
         };

         template<class T>
         struct ParallelComputeAccFromMultiMapping
         {
           void operator()(void *m, Shared_rw< defaulttype::SharedVector<typename T::Out::VecDeriv*> > acc_out, Shared_r<typename T::In::VecDeriv> v_in,Shared_r<typename T::In::VecDeriv> acc_in){
             ((T *)m)->::computeAccFromMapping(acc_out.access(), v_in.read(), acc_in.read());
           }
         };


         template <class In, class Out>
         struct ParallelComputeAccFromMultiMapping< MechanicalMultiMapping<In, Out> >
         {
                 void operator()(MechanicalMultiMapping<In,Out> *m,Shared_rw< defaulttype::SharedVector<typename Out::VecDeriv*> > acc_out, Shared_r<defaulttype::SharedVector<const typename In::VecDeriv*> > v_in,Shared_r<defaulttype::SharedVector<const typename In::VecDeriv* > > acc_in){
                         m->computeAccFromMapping(acc_out.access(),v_in.read(),acc_in.read());
                 }
         };

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateX()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                return;

             const VecId &idCoord = VecId::position();
             VecOutPos.resize(0);

             getVecOutCoord(idCoord, VecOutPos);
             VecInPos.resize(0);
             getConstVecInCoord(idCoord, VecInPos);

             Task<ParallelMultiMappingApplyCPU3< MultiMapping<In,Out> >,ParallelMultiMappingApply3< MultiMapping<In,Out> > >(this,**(VecOutPos[0]),**(VecOutPos[0]), **(VecInPos[1]));
             //Task<ParallelMultiMappingApplyCPU< MultiMapping<In,Out> >,ParallelMultiMappingApply< MultiMapping<In,Out> > >(this,*VecOutPos, *VecInPos);
         }

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateXfree()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             const VecId &idDeriv = VecId::freePosition();
             VecOutFreeVel.resize(0);
             getVecOutDeriv(idDeriv, VecOutFreeVel);
             VecInFreeVel.resize(0);
             getConstVecInDeriv(idDeriv, VecInFreeVel);

             Task<ParallelMultiMappingApplyCPU3< MultiMapping<In,Out> >,ParallelMultiMappingApply3< MultiMapping<In,Out> > >(this,**(VecOutFreeVel[0]),**(VecInFreeVel[0]), **(VecInFreeVel[1]));
             //Task<ParallelMultiMappingApplyCPU< MultiMapping<In,Out> >,ParallelMultiMappingApply< MultiMapping<In,Out> > >(this,*VecOutFreeVel, *VecInFreeVel);
         }

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateDx()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             const VecId &idDeriv = VecId::dx();
             VecOutDx.resize(0);
             getVecOutDeriv(idDeriv, VecOutDx);
             VecInDx.resize(0);
             getConstVecInDeriv(idDeriv, VecInDx);

             Task<ParallelMultiMappingApplyJCPU3< MultiMapping<In,Out> >,ParallelMultiMappingApplyJ3< MultiMapping<In,Out> > >(this,**VecOutDx[0], **VecInDx[0],**VecInDx[1]);
             //Task<ParallelMultiMappingApplyJCPU< MultiMapping<In,Out> >,ParallelMultiMappingApplyJ< MultiMapping<In,Out> > >(this,*VecOutDx, *VecInDx);
         }

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateV()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             const VecId &idDeriv = VecId::velocity();
             VecOutVel.resize(0);
             getVecOutDeriv(idDeriv, VecOutVel);
             VecInVel.resize(0);
             getConstVecInDeriv(idDeriv, VecInVel);

             Task<ParallelMultiMappingApplyJCPU3< MultiMapping<In,Out> >,ParallelMultiMappingApplyJ3< MultiMapping<In,Out> > >(this,**VecOutVel[0], **VecInVel[0],**VecInVel[1]);
             //Task<ParallelMultiMappingApplyJCPU< MultiMapping<In,Out> >,ParallelMultiMappingApplyJ< MultiMapping<In,Out> > >(this,*VecOutVel, *VecInVel);
         }

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::propagateA()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             const VecId &v = VecId::velocity();
             const VecId &dx = VecId::dx();

             VecOutDx2.resize(0);
             getVecOutDeriv(dx, VecOutDx2);
             VecInVel2.resize(0);
             getConstVecInDeriv(v, VecInVel2);
             VecInDx2.resize(0);
             getConstVecInDeriv(dx, VecInDx2);

             Task<ParallelComputeAccFromMultiMapping3<MechanicalMultiMapping <In,Out> > >(this,**VecOutDx2[0], **VecInVel2[0], **VecInVel2[1], **VecInDx2[0], **VecInDx2[1] );
             //Task<ParallelComputeAccFromMultiMapping<MechanicalMultiMapping <In,Out> > >(this,*VecOutDx2, *VecInVel2, *VecInDx2 );
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



         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::accumulateForce()
         {

             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             VecInForce.resize(0);
             GetForceVectorFunctor<In, defaulttype::SharedVector<typename In::VecDeriv*> > InF(VecInForce);
             std::for_each(this->fromModels.begin(), this->fromModels.end(), InF);

             VecOutForce.resize(0);
             GetForceVectorFunctor<Out, defaulttype::SharedVector<const typename Out::VecDeriv*> > OutF(VecOutForce);
             std::for_each(this->toModels.begin(), this->toModels.end(), OutF);

             Task<ParallelMultiMappingApplyJTCPU3< MechanicalMultiMapping<In,Out> >,ParallelMultiMappingApplyJT3< MechanicalMultiMapping<In,Out> > >(this,**VecInForce[0],**VecInForce[1], **VecOutForce[0]);
             //Task<ParallelMultiMappingApplyJTCPU< MechanicalMultiMapping<In,Out> >,ParallelMultiMappingApplyJT< MechanicalMultiMapping<In,Out> > >(this,*VecInForce, *VecOutForce);

         }

         template <class In, class Out>
         void MechanicalMultiMapping<In,Out>::accumulateDf()
         {
             if( this->fromModels.empty() || this->toModels.empty() )
                 return;

             // in SOFA_SMP  this->fromModels[i]->getForceId(); return a vecid > 9
             VecInForce2.resize(this->fromModels.size());
             for (unsigned int i=0;i<this->fromModels.size();++i)
             {
                 // don't solve totaly the problem
                 std::cout << this->fromModels[i]->getForceId() << std::endl;
                 const VecId &f = VecId::force();//this->fromModels[i]->getForceId();
                 std::cout << f << std::endl;
                 VecInForce2[i]=this->fromModels[i]->getVecDeriv(f.index);
             }

             VecOutForce2.resize(this->toModels.size());
             for (unsigned int i=0;i<this->toModels.size();++i)
             {
                 const VecId &f = VecId::internalForce();//this->toModels[i]->getForceId();
                 std::cout << f << std::endl;
                 VecOutForce2[i]=this->toModels[i]->getVecDeriv(f.index);
             }

             /*VecInForce2.resize(0);
             GetForceVectorFunctor<In, defaulttype::SharedVector<typename In::VecDeriv*> > InF(VecInForce2);
             std::for_each(this->fromModels.begin(), this->fromModels.end(), InF);

             VecOutForce2.resize(0);
             GetForceVectorFunctor<Out, defaulttype::SharedVector<const typename Out::VecDeriv*> > OutF(VecOutForce2);
             std::for_each(this->toModels.begin(), this->toModels.end(), OutF);*/

             Task<ParallelMultiMappingApplyJTCPU2< MechanicalMultiMapping<In,Out> >,ParallelMultiMappingApplyJT2< MechanicalMultiMapping<In,Out> > >(this,**VecInForce2[0],**VecInForce2[1], **VecOutForce2[0]);
             //Task<ParallelMultiMappingApplyJTCPU< MechanicalMultiMapping<In,Out> >,ParallelMultiMappingApplyJT< MechanicalMultiMapping<In,Out> > >(this,*VecInForce2, *VecOutForce2);
         }

         #endif // SOFA_SMP

} // namespace behavior

} // namespace core

} // namespace sofa

#endif

