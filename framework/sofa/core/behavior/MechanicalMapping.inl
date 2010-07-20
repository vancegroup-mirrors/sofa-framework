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
#ifndef SOFA_CORE_BEHAVIOR_MECHANICALMAPPING_INL
#define SOFA_CORE_BEHAVIOR_MECHANICALMAPPING_INL

#include <sofa/core/Mapping.h>
#include <sofa/core/Mapping.inl>
#include <sofa/core/behavior/MechanicalMapping.h>
#include <iostream>

namespace sofa
{

namespace core
{

namespace behavior
{


template <class In, class Out>
MechanicalMapping<In,Out>::MechanicalMapping(In* from, Out* to)
: Mapping<In,Out>(from, to)
, f_isMechanical( initData( &f_isMechanical, true, "isMechanical", "set to false if this mapping should only be used as a regular mapping instead of a mechanical mapping" ) )
{
}

template <class In, class Out>
MechanicalMapping<In,Out>::~MechanicalMapping()
{
}

template <class In, class Out>
BaseMechanicalState* MechanicalMapping<In,Out>::getMechFrom()
{
	return this->fromModel;
}

template <class In, class Out>
BaseMechanicalState* MechanicalMapping<In,Out>::getMechTo()
{
	return this->toModel;
}

template <class In, class Out>
bool MechanicalMapping<In,Out>::isMechanical()
{
    return this->f_isMechanical.getValue();
}

template <class In, class Out>
void MechanicalMapping<In,Out>::setMechanical(bool b)
{
  f_isMechanical.setValue(b);
}

template <class In, class Out>
void MechanicalMapping<In,Out>::init()
{
#ifdef SOFA_SMP
	if (this->toModel == NULL || this->fromModel == NULL)
		return;

	if (this->toModel->getX()!=NULL && this->fromModel->getX()!=NULL)
	{
		apply(*this->toModel->getX(), *this->fromModel->getX());
		//cerr<<"Mapping<In,Out>::updateMapping(), *this->fromModel->getX() = "<<*this->fromModel->getX()<<endl;
		//cerr<<"Mapping<In,Out>::updateMapping(), *this->toModel->getX() = "<<*this->toModel->getX()<<endl;
	}
	if (this->toModel->getV()!=NULL && this->fromModel->getV()!=NULL)
	{
		applyJ(*this->toModel->getV(), *this->fromModel->getV());
	}
    if (this->fromModel!=NULL && this->toModel->getXfree()!=NULL && this->fromModel->getXfree()!=NULL)
		apply(*this->toModel->getXfree(), *this->fromModel->getXfree());
#else
    this->updateMapping();
    propagateXfree();
#endif
}

#ifndef SOFA_SMP
template <class In, class Out>
void MechanicalMapping<In,Out>::propagateX()
{
    if (this->fromModel!=NULL && this->toModel->getX()!=NULL && this->fromModel->getX()!=NULL)
		apply(*this->toModel->getX(), *this->fromModel->getX());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateV()
{
    if (this->fromModel!=NULL && this->toModel->getV()!=NULL && this->fromModel->getV()!=NULL)
		applyJ(*this->toModel->getV(), *this->fromModel->getV());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateA()
{
    if (this->fromModel!=NULL && this->toModel->getDx()!=NULL && this->fromModel->getV()!=NULL &&  this->fromModel->getDx()!=NULL )
		computeAccFromMapping(*this->toModel->getDx(), *this->fromModel->getV(), *this->fromModel->getDx());
}


template <class In, class Out>
void MechanicalMapping<In,Out>::propagateDx()
{
    if (this->fromModel!=NULL && this->toModel->getDx()!=NULL && this->fromModel->getDx()!=NULL)
		applyJ(*this->toModel->getDx(), *this->fromModel->getDx());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateXfree()
{
    if (this->fromModel!=NULL && this->toModel->getXfree()!=NULL && this->fromModel->getXfree()!=NULL)
		apply(*this->toModel->getXfree(), *this->fromModel->getXfree());
}


template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateForce()
{
/*    if( this->fromModel==NULL ) serr<<"MechanicalMapping<In,Out>::accumulateForce, toModel is NULL"<<sendl;
    else if( this->toModel==NULL ) serr<<"MechanicalMapping<In,Out>::accumulateForce, toModel is NULL"<<sendl;
    else serr<<"MechanicalMapping<In,Out>::accumulateForce() OK"<<sendl;*/
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
		applyJT(*this->fromModel->getF(), *this->toModel->getF());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateDf()
{
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
		applyJT(*this->fromModel->getF(), *this->toModel->getF());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateConstraint()
{
    if (this->fromModel!=NULL && this->toModel->getC()!=NULL && this->fromModel->getC()!=NULL)
	{
		applyJT(*this->fromModel->getC(), *this->toModel->getC());

		// Accumulate contacts indices through the MechanicalMapping
		std::vector<unsigned int>::iterator it = this->toModel->getConstraintId().begin();
		std::vector<unsigned int>::iterator itEnd = this->toModel->getConstraintId().end();

		while (it != itEnd)
		{
			this->fromModel->setConstraintId(*it);
			it++;
		}
	}
}
#else
using sofa::core::ParallelMappingApply;
using sofa::core::ParallelMappingApplyJ;


/************* Mapping Functors *********************/

template <class T>
struct ParallelMappingApplyJT
{
  void operator()(void *m, Shared_rw< typename T::In::VecDeriv> out, Shared_r<typename T::Out::VecDeriv> in){
    ((T *)m)->applyJT(out.access(), in.read());
  }
};
template <class T>
struct ParallelMappingApplyJTCPU
{
  void operator()(void *m, Shared_rw< typename T::In::VecDeriv> out, Shared_r<typename T::Out::VecDeriv> in){
    ((T *)m)->applyJT(out.access(), in.read());
  }
};

template<class T>
struct ParallelComputeAccFromMapping
{
  void operator()(void *m, Shared_rw< typename T::Out::VecDeriv> acc_out, Shared_r<typename T::In::VecDeriv> v_in,Shared_r<typename T::In::VecDeriv> acc_in){
    ((T *)m)->::computeAccFromMapping(acc_out.access(), v_in.read(), acc_in.read());
  }
};

template <class In, class Out>
struct ParallelComputeAccFromMapping< MechanicalMapping<In, Out> >
{
	void operator()(MechanicalMapping<In,Out> *m,Shared_rw< typename Out::VecDeriv> acc_out, Shared_r<typename In::VecDeriv> v_in,Shared_r<typename In::VecDeriv> acc_in){
		m->computeAccFromMapping(acc_out.access(),v_in.read(),acc_in.read());
	}
};

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateX()
{
if (this->fromModel!=NULL && this->toModel->getX()!=NULL && this->fromModel->getX()!=NULL)
        Task<ParallelMappingApplyCPU< Mapping<In,Out> >,ParallelMappingApply< Mapping<In,Out> > >(this,**this->toModel->getX(), **this->fromModel->getX());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateV()
{
    if (this->fromModel!=NULL && this->toModel->getV()!=NULL && this->fromModel->getV()!=NULL)
        Task<ParallelMappingApplyJCPU< Mapping<In,Out> >,ParallelMappingApplyJ< Mapping<In,Out> > >(this,**this->toModel->getV(), **this->fromModel->getV());
}
template <class In, class Out>
void MechanicalMapping<In,Out>::propagateA()
{
//    if (this->fromModel!=NULL && this->toModel->getDx()!=NULL && this->fromModel->getV()!=NULL &&  this->fromModel->getDx()!=NULL )
//		Task<ParallelComputeAccFromMapping<MechanicalMapping <In,Out> > >(this,**this->toModel->getDx(), **this->fromModel->getV(), **this->fromModel->getDx());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateDx()
{
    if (this->fromModel!=NULL && this->toModel->getDx()!=NULL && this->fromModel->getDx()!=NULL)
        Task<ParallelMappingApplyJCPU< Mapping<In,Out> >,ParallelMappingApplyJ< Mapping<In,Out> > >(this,**this->toModel->getDx(), **this->fromModel->getDx());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::propagateXfree()
{
    if (this->fromModel!=NULL && this->toModel->getXfree()!=NULL && this->fromModel->getXfree()!=NULL)
        Task<ParallelMappingApplyCPU< Mapping<In,Out> >,ParallelMappingApply< Mapping<In,Out> > >(this,**this->toModel->getXfree(), **this->fromModel->getXfree());
}


template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateForce()
{
/*    if( this->fromModel==NULL ) cerr<<"MechanicalMapping<In,Out>::accumulateForce, toModel is NULL"<<endl;
    else if( this->toModel==NULL ) cerr<<"MechanicalMapping<In,Out>::accumulateForce, toModel is NULL"<<endl;
    else cerr<<"MechanicalMapping<In,Out>::accumulateForce() OK"<<endl;*/
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
        Task<ParallelMappingApplyJTCPU< MechanicalMapping<In,Out> >,ParallelMappingApplyJT< MechanicalMapping<In,Out> > >(this,**this->fromModel->getF(), **this->toModel->getF());

}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateDf()
{
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
        Task<ParallelMappingApplyJTCPU< MechanicalMapping<In,Out> >,ParallelMappingApplyJT< MechanicalMapping<In,Out> > >(this,**this->fromModel->getF(), **this->toModel->getF());
}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateConstraint()
{
    if (this->fromModel!=NULL && this->toModel->getC()!=NULL && this->fromModel->getC()!=NULL)
	{
		applyJT(*this->fromModel->getC(), *this->toModel->getC());
	       // Task<ParallelMappingApplyJTCPU<MechanicalMapping<In,Out> >,ParallelMappingApplyJT<MechanicalMapping<In,Out> >  >(this,**this->fromModel->getC(), **this->toModel->getC());
		// Accumulate contacts indices through the MechanicalMapping
		std::vector<unsigned int>::iterator it = this->toModel->getConstraintId().begin();
		std::vector<unsigned int>::iterator itEnd = this->toModel->getConstraintId().end();

		while (it != itEnd)
		{
			this->fromModel->setConstraintId(* it);
			it++;
		}
	}
}
#endif


} // namespace behavior

} // namespace core

} // namespace sofa

#endif
