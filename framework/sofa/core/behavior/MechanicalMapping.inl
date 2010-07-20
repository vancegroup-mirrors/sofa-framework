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
#ifndef SOFA_SMP
, f_checkJacobian( initData( &f_checkJacobian, false, "checkJacobian", "set to true to compare results of applyJ/applyJT methods with multiplication with the matrix given by getJ()" ) )
#endif
{
}

template <class In, class Out>
MechanicalMapping<In,Out>::~MechanicalMapping()
{
}

template <class In, class Out>
std::string MechanicalMapping<In,Out>::getTemplateName() const
{
  return templateName(this);
}

template <class In, class Out>
helper::vector<BaseMechanicalState*> MechanicalMapping<In,Out>::getMechFrom()
{
  helper::vector<BaseMechanicalState*> vec(1,this->fromModel);
	return vec;
}

template <class In, class Out>
helper::vector<BaseMechanicalState*> MechanicalMapping<In,Out>::getMechTo()
{
  helper::vector<BaseMechanicalState*> vec(1,this->toModel);
	return vec;
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
    {
        if (f_checkJacobian.getValue())
            checkApplyJ(*this->toModel->getV(), *this->fromModel->getV(), this->getJ());
        else
            applyJ(*this->toModel->getV(), *this->fromModel->getV());
    }
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
    {
        if (f_checkJacobian.getValue())
            checkApplyJ(*this->toModel->getDx(), *this->fromModel->getDx(), this->getJ());
        else
            applyJ(*this->toModel->getDx(), *this->fromModel->getDx());
    }
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
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
    {
        if (f_checkJacobian.getValue())
            checkApplyJT(*this->fromModel->getF(), *this->toModel->getF(), this->getJ());
        else
            applyJT(*this->fromModel->getF(), *this->toModel->getF());
    }
}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateDf()
{
    if (this->fromModel!=NULL && this->toModel->getF()!=NULL && this->fromModel->getF()!=NULL)
    {
        if (f_checkJacobian.getValue())
            checkApplyJT(*this->fromModel->getF(), *this->toModel->getF(), this->getJ());
        else
            applyJT(*this->fromModel->getF(), *this->toModel->getF());
    }
}

template <class In, class Out>
void MechanicalMapping<In,Out>::accumulateConstraint()
{
    if (this->fromModel!=NULL && this->toModel->getC()!=NULL && this->fromModel->getC()!=NULL)
	{
        if (f_checkJacobian.getValue())
            checkApplyJT(*this->fromModel->getC(), *this->toModel->getC(), this->getJ());
        else
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

template <class In, class Out>
bool MechanicalMapping<In,Out>::checkApplyJ( OutVecDeriv& out, const InVecDeriv& in, const sofa::defaulttype::BaseMatrix* J )
{
    applyJ(out, in);
    if (!J)
    {
        serr << "CheckApplyJ: getJ returned a NULL matrix" << sendl;
        return false;
    }

    if (this->toModel->forceMask.isInUse())
    {
        serr << "Mask in use in mapped model. Disabled because of checkApplyJ." << sendl;
        this->toModel->forceMask.setInUse(false);
    }

    OutVecDeriv out2;
    out2.resize(out.size());

    matrixApplyJ(out2, in, J);

    // compare out and out2
    const int NOut = sofa::defaulttype::DataTypeInfo<typename Out::Deriv>::Size;
    double diff_mean = 0, diff_max = 0, val1_mean = 0, val2_mean = 0;
    for (unsigned int i=0;i<out.size();++i)
        for (int j=0;j<NOut;++j)
        {
            double v1 = out[i][j];
            double v2 = out2[i][j];
            double diff = v1-v2;
            if (diff < 0) diff = -diff;
            if (diff > diff_max) diff_max = diff;
            diff_mean += diff;
            if (v1 < 0) v1=-v1;
            val1_mean += v1;
            if (v2 < 0) v2=-v2;
            val2_mean += v2;
        }
    diff_mean /= out.size() * NOut;
    val1_mean /= out.size() * NOut;
    val2_mean /= out.size() * NOut;
    sout << "Comparison of applyJ() and matrix from getJ(): ";
    sout << "Max Error = " << diff_max;
    sout << "\t Mean Error = " << diff_mean;
    sout << "\t Mean Abs Value from applyJ = " << val1_mean;
    sout << "\t Mean Abs Value from matrix = " << val2_mean;
    sout << sendl;
    if (this->f_printLog.getValue() || diff_max > 0.1*(val1_mean+val2_mean)/2)
    {
        sout << "Input vector       : " << in << sendl;
        sout << "Result from applyJ : " << out << sendl;
        sout << "Result from matrix : " << out2 << sendl;
    }
    return true;
}

template <class In, class Out>
void MechanicalMapping<In,Out>::matrixApplyJ( OutVecDeriv& out, const InVecDeriv& in, const sofa::defaulttype::BaseMatrix* J )
{
    typedef typename Out::Real OutReal;
    typedef typename In::Real InReal;
    typedef typename Out::Deriv OutDeriv;
    typedef typename In::Deriv InDeriv;
    if (!J) return;
    if (J->rowSize() == 0) return;
    const int NIn = sofa::defaulttype::DataTypeInfo<InDeriv>::Size;
    const int NOut = sofa::defaulttype::DataTypeInfo<OutDeriv>::Size;
    out.resize(J->rowSize() / NOut);
    OutReal* in_alloc = NULL;
    OutReal* out_alloc = NULL;
    const OutReal* in_buffer = NULL;
    OutReal* out_buffer = NULL;
    if (sizeof(InReal) == sizeof(OutReal) && sofa::defaulttype::DataTypeInfo<InDeriv>::SimpleLayout)
    { // we can use the data directly
        in_buffer = (const OutReal*)&in[0];
    }
    else
    { // we must copy the values
        in_alloc = new OutReal[in.size()*NIn];
        for (unsigned int i=0;i<in.size();++i)
            for (int j=0;j<NIn;++j)
                in_alloc[i*NIn+j] = (OutReal)in[i][j];
        in_buffer = in_alloc;
    }
    if (sofa::defaulttype::DataTypeInfo<OutDeriv>::SimpleLayout)
    { // we can use the data directly
        out_buffer = (OutReal*)&out[0];
    }
    else
    { // we must copy the values
        out_alloc = new OutReal[out.size()*NOut];
        for (unsigned int i=0;i<out.size();++i)
            for (int j=0;j<NOut;++j)
                out_alloc[i*NOut+j] = (OutReal)0; //out[i][j];
        out_buffer = out_alloc;
    }
    // Do the matrix multiplication
    J->opMulV(out_buffer, in_buffer);
    if (in_alloc)
    {
        delete[] in_alloc;
    }
    if (out_alloc)
    {
        for (unsigned int i=0;i<out.size();++i)
            for (int j=0;j<NOut;++j)
                out[i][j] = out_alloc[i*NOut+j];
        delete[] out_alloc;
    }
}

template <class In, class Out>
bool MechanicalMapping<In,Out>::checkApplyJT( InVecDeriv& out, const OutVecDeriv& in, const sofa::defaulttype::BaseMatrix* J )
{
    if (!J)
    {
        serr << "CheckApplyJT: getJ returned a NULL matrix" << sendl;
        applyJT(out, in);
        return false;
    }

    if (this->toModel->forceMask.isInUse())
    {
        serr << "Mask in use in mapped model. Disabled because of checkApplyJT." << sendl;
        this->toModel->forceMask.setInUse(false);
    }

    InVecDeriv tmp;
    tmp.resize(out.size());
    applyJT(tmp, in);
    if (tmp.size() > out.size())
        out.resize(tmp.size());
    for (unsigned int i=0;i<tmp.size();++i)
        out[i] += tmp[i];

    InVecDeriv tmp2;
    tmp2.resize(out.size());

    matrixApplyJT(tmp2, in, J);

    // compare tmp and tmp2
    const int NOut = sofa::defaulttype::DataTypeInfo<typename Out::Deriv>::Size;
    double diff_mean = 0, diff_max = 0, val1_mean = 0, val2_mean = 0;
    for (unsigned int i=0;i<tmp.size();++i)
        for (int j=0;j<NOut;++j)
        {
            double v1 = tmp[i][j];
            double v2 = tmp2[i][j];
            double diff = v1-v2;
            if (diff < 0) diff = -diff;
            if (diff > diff_max) diff_max = diff;
            diff_mean += diff;
            if (v1 < 0) v1=-v1;
            val1_mean += v1;
            if (v2 < 0) v2=-v2;
            val2_mean += v2;
        }
    diff_mean /= tmp.size() * NOut;
    val1_mean /= tmp.size() * NOut;
    val2_mean /= tmp.size() * NOut;
    sout << "Comparison of applyJT() and matrix^T from getJ(): ";
    sout << "Max Error = " << diff_max;
    sout << "\t Mean Error = " << diff_mean;
    sout << "\t Mean Abs Value from applyJT = " << val1_mean;
    sout << "\t Mean Abs Value from matrixT = " << val2_mean;
    sout << sendl;
    if (this->f_printLog.getValue() || diff_max > 0.1*(val1_mean+val2_mean)/2)
    {
        sout << "Input vector        : " << in << sendl;
        sout << "Result from applyJT : " << tmp << sendl;
        sout << "Result from matrixT : " << tmp2 << sendl;
    }
    return true;
}

template <class In, class Out>
void MechanicalMapping<In,Out>::matrixApplyJT( InVecDeriv& out, const OutVecDeriv& in, const sofa::defaulttype::BaseMatrix* J )
{
    typedef typename Out::Real OutReal;
    typedef typename In::Real InReal;
    typedef typename Out::Deriv OutDeriv;
    typedef typename In::Deriv InDeriv;
    if (!J) return;
    if (J->rowSize() == 0) return;
    const int NIn = sofa::defaulttype::DataTypeInfo<InDeriv>::Size;
    const int NOut = sofa::defaulttype::DataTypeInfo<OutDeriv>::Size;
    out.resize(J->colSize() / NOut);
    InReal* in_alloc = NULL;
    InReal* out_alloc = NULL;
    const InReal* in_buffer = NULL;
    InReal* out_buffer = NULL;
    if (sofa::defaulttype::DataTypeInfo<OutDeriv>::SimpleLayout)
    { // we can use the data directly
        in_buffer = (const InReal*)&in[0];
    }
    else
    { // we must copy the values
        in_alloc = new InReal[in.size()*NOut];
        for (unsigned int i=0;i<in.size();++i)
            for (int j=0;j<NOut;++j)
                in_alloc[i*NOut+j] = (InReal)in[i][j];
        in_buffer = in_alloc;
    }
    if (sizeof(InReal) == sizeof(OutReal) && sofa::defaulttype::DataTypeInfo<InDeriv>::SimpleLayout)
    { // we can use the data directly
        out_buffer = (InReal*)&out[0];
    }
    else
    { // we must copy the values
        out_alloc = new InReal[out.size()*NIn];
        for (unsigned int i=0;i<out.size();++i)
            for (int j=0;j<NIn;++j)
                out_alloc[i*NIn+j] = (InReal)0; //out[i][j];
        out_buffer = out_alloc;
    }
    // Do the transposed matrix multiplication
    J->opPMulTV(out_buffer, in_buffer);
    if (in_alloc)
    {
        delete[] in_alloc;
    }
    if (out_alloc)
    {
        for (unsigned int i=0;i<out.size();++i)
            for (int j=0;j<NIn;++j)
                out[i][j] += out_alloc[i*NIn+j];
        delete[] out_alloc;
    }
}

template <class In, class Out>
bool MechanicalMapping<In,Out>::checkApplyJT( InVecConst& out, const OutVecConst& in, const sofa::defaulttype::BaseMatrix* J )
{
    applyJT(out, in);
    if (!J)
    {
        serr << "CheckApplyJT: getJ returned a NULL matrix" << sendl;
        return false;
    }

    return true;
}

template <class In, class Out>
void MechanicalMapping<In,Out>::matrixApplyJT( InVecConst& /*out*/, const OutVecConst& /*in*/, const sofa::defaulttype::BaseMatrix* /*J*/ )
{
    serr << "matrixApplyJT for VecConst NOT IMPLEMENTED" << sendl;
}
#else // SOFA_SMP

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
#endif // SOFA_SMP


} // namespace behavior

} // namespace core

} // namespace sofa

#endif
