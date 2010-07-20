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
#ifndef SOFA_CORE_BEHAVIOR_MECHANICALMAPPING_H
#define SOFA_CORE_BEHAVIOR_MECHANICALMAPPING_H

#include <sofa/core/Mapping.h>
#include <sofa/core/behavior/BaseMechanicalMapping.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Specialized interface to convert a mechanical model of type In to
 *  another mechanical model of type Out.
 *
 *  A MechanicalMapping is a mapping between two MechanicalState.
 *  Position, velocity and displacement are propagated from the source (upper)
 *  model to the destination (lower, or mapped) model, while forces and
 *  constraints are accumulated back from the destination model to the source
 *  model.
 *
 *  If the mapping is linear, then it can be represented by a matrix J, and
 *  most computations correspond to multiply this matrix or its transpose to
 *  the given vectors.
 */
template <class TIn, class TOut>
class MechanicalMapping : public Mapping<TIn,TOut>, public BaseMechanicalMapping
{
public:
    SOFA_CLASS2(SOFA_TEMPLATE2(MechanicalMapping,TIn,TOut), SOFA_TEMPLATE2(Mapping,TIn,TOut), BaseMechanicalMapping);

    /// Input Model Type
    typedef TIn In;
    /// Output Model Type
    typedef TOut Out;

    typedef typename In::VecCoord InVecCoord;
    typedef typename In::VecDeriv InVecDeriv;
    typedef typename In::VecConst InVecConst;

    typedef typename Out::VecCoord OutVecCoord;
    typedef typename Out::VecDeriv OutVecDeriv;
    typedef typename Out::VecConst OutVecConst;

    Data<bool> f_isMechanical;
#ifndef SOFA_SMP
    Data<bool> f_checkJacobian;
#endif

    MechanicalMapping(In* from, Out* to);

    virtual ~MechanicalMapping();

    /// Apply the reverse mapping on force vectors.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ out += J^t in $
    ///
    /// This method must be reimplemented by all mappings.
    virtual void applyJT( InVecDeriv& out, const OutVecDeriv& in ) = 0;

    /// Apply the reverse mapping on constraint matrix.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ Out += J^t In $
    ///
    /// This method must be reimplemented by all mappings if they need to support constraints.
    virtual void applyJT( InVecConst& /*out*/, const OutVecConst& /*in*/ ) {}

    /// If the mapping input has a rotation velocity, it computes the subsequent acceleration
    /// created by the derivative terms
    /// $ a_out = w^(w^rel_pos)	$
    /// This method must be reimplemented by all mappings if they need to support composite accelerations
    virtual void computeAccFromMapping(  OutVecDeriv& /*acc_out*/, const InVecDeriv& /*v_in*/, const InVecDeriv& /*acc_in*/){}

        
    /// Get the source (upper) model.
    virtual helper::vector<BaseMechanicalState*> getMechFrom();

    /// Get the destination (lower, mapped) model.
    virtual helper::vector<BaseMechanicalState*> getMechTo();

    /// Return false if this mapping should only be used as a regular mapping instead of a mechanical mapping.
    virtual bool isMechanical();

    /// Determine if this mapping should only be used as a regular mapping instead of a mechanical mapping.
    virtual void setMechanical(bool b);

    virtual void init();

    /// Propagate position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ x_out = J x_in $
    ///
    /// This method retrieves the x vectors and call the internal apply() method implemented by the component.
    virtual void propagateX();

    /// Propagate free-motion position from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ xfree_out = J xfree_in $
    ///
    /// This method retrieves the xfree vectors and call the internal apply() method implemented by the component.
    virtual void propagateXfree();

    /// Propagate velocity from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ v_out = J v_in $
    ///
    /// This method retrieves the v vectors and call the internal apply() method implemented by the component.
    virtual void propagateV();

	/// Propagate acceleration due to the derivative of the mapping function
	///
	/// If the mapping input has a rotation velocity, it computes the subsequent acceleration
	/// $ a_out = w^(w^rel_pos)	$
	///
	/// This method retrieves the acc and v vectors and call the internal computeAccFromMapping() method implemented by the component.
    virtual void propagateA();


    /// Propagate displacement from the source model to the destination model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ dx_out = J dx_in $
    ///
    /// This method retrieves the dx vectors and call the internal applyJ() method implemented by the component.
    virtual void propagateDx();

    /// Accumulate forces from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ f_in += J^t f_out $
    ///
    /// This method retrieves the force vectors and call the internal applyJT() method implemented by the component.
    virtual void accumulateForce();

    /// Accumulate force deltas from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ df_in += J^t df_out $
    ///
    /// This method retrieves the force vectors and call the internal applyJT() method implemented by the component.
    virtual void accumulateDf();

    /// Accumulate constraint from the destination model back to the source model.
    ///
    /// If the MechanicalMapping can be represented as a matrix J, this method computes
    /// $ C_in += J^t C_out $
    ///
    /// This method retrieves the constraint matrices and call the internal applyJT() method implemented by the component.
    virtual void accumulateConstraint();


    virtual std::string getTemplateName() const;

    static std::string templateName(const MechanicalMapping<In, Out>* = NULL)
    {
      return std::string("MechanicalMapping<")+In::DataTypes::Name() + std::string(",") + Out::DataTypes::Name() + std::string(">");
    }

protected:
    bool getShow() const { return this->getContext()->getShowMechanicalMappings(); }

#ifndef SOFA_SMP
    void matrixApplyJ( OutVecDeriv& out, const InVecDeriv& in, const sofa::defaulttype::BaseMatrix* J );
    void matrixApplyJT( InVecDeriv& out, const OutVecDeriv& in, const sofa::defaulttype::BaseMatrix* J );
    void matrixApplyJT( InVecConst& out, const OutVecConst& in, const sofa::defaulttype::BaseMatrix* J );
    bool checkApplyJ( OutVecDeriv& out, const InVecDeriv& in, const sofa::defaulttype::BaseMatrix* J );
    bool checkApplyJT( InVecDeriv& out, const OutVecDeriv& in, const sofa::defaulttype::BaseMatrix* J );
    bool checkApplyJT( InVecConst& out, const OutVecConst& in, const sofa::defaulttype::BaseMatrix* J );
#endif

};

#if defined(WIN32) && !defined(SOFA_BUILD_CORE)

using namespace sofa::defaulttype;
using namespace sofa::core::behavior;

extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec3dTypes>, MechanicalState<Vec3dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec2dTypes>, MechanicalState<Vec2dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Vec1dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec6dTypes>, MechanicalState<Vec6dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid2dTypes>, MechanicalState<Vec2dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Rigid2dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid3dTypes>, MechanicalState<Vec3dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Rigid3dTypes> > ;

extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Vec1fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec2fTypes>, MechanicalState<Vec2fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec3fTypes>, MechanicalState<Vec3fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec6fTypes>, MechanicalState<Vec6fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid3fTypes>, MechanicalState<Vec3fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid2fTypes>, MechanicalState<Vec2fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Rigid2fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Rigid3fTypes> > ;

extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec3dTypes>, MechanicalState<Vec3fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec3fTypes>, MechanicalState<Vec3dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Vec1fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Vec1dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec2dTypes>, MechanicalState<Vec2fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec2fTypes>, MechanicalState<Vec2dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec6dTypes>, MechanicalState<Vec6fTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec6fTypes>, MechanicalState<Vec6dTypes> >;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid2dTypes>, MechanicalState<Vec2fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid2fTypes>, MechanicalState<Vec2dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Rigid2fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Rigid2dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid3dTypes>, MechanicalState<Vec3fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Rigid3fTypes>, MechanicalState<Vec3dTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1dTypes>, MechanicalState<Rigid3fTypes> > ;
extern template class SOFA_CORE_API MechanicalMapping< MechanicalState<Vec1fTypes>, MechanicalState<Rigid3dTypes> > ;
#endif

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
