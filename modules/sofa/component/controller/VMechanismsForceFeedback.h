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
#ifndef SOFA_COMPONENT_CONTROLLER_NLCPFORCEFEEDBACK_H
#define SOFA_COMPONENT_CONTROLLER_NLCPFORCEFEEDBACK_H

#include <sofa/component/component.h>
#include <sofa/component/controller/ForceFeedback.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/component/mastersolver/MasterConstraintSolver.h>
#include <sofa/component/mastersolver/MasterContactSolver.h>

namespace sofa
{

    namespace component
    {
	namespace odesolver 
	{
            class MasterContactSolver;
            class LCP;
	}
	namespace mastersolver
	{
            class MasterConstraintSolver;
            class ConstraintProblem;
	}

    namespace controller
    {
        using namespace std;
        using namespace helper::system::thread;
		using namespace core::behavior;
		using namespace core;

        /**
* VMechanisms_ForceField
*/
        template <class TDataTypes>
                class SOFA_COMPONENT_CONTROLLER_API VMechanismsForceFeedback : public sofa::component::controller::ForceFeedback
        {

        public:
            SOFA_CLASS(SOFA_TEMPLATE(VMechanismsForceFeedback,TDataTypes),sofa::component::controller::ForceFeedback);

            typedef TDataTypes DataTypes;
            typedef typename DataTypes::Coord Coord;
            typedef typename DataTypes::Deriv Deriv;
            typedef typename DataTypes::MatrixDeriv MatrixDeriv;
            typedef typename DataTypes::VecDeriv VecDeriv;
            typedef typename DataTypes::VecCoord VecCoord;

            typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
            typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;
            typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
            typedef typename DataTypes::MatrixDeriv::ColIterator MatrixDerivColIterator;

            //??? is to be templated at all??? :
            typedef defaulttype::SparseConstraint<typename DataTypes::Deriv> SparseConstraint;
            typedef typename SparseConstraint::const_data_iterator ConstraintIterator;

            void init();

            void draw()
            {

                // draw the haptic_freq in the openGL window
              //  std::cout << "haptic_freq = " << std::fixed << haptic_freq << " Hz   " << '\xd';
            //haptic_freq += 0.000001;

			//sofa::helper::gl::glfntInit();
			//char[4] _toto = "toto";
			//sofa::helper::gl::glfntWriteBitmap(0.0f,0.0f,_toto);
			//sofa::helper::gl::glfntClose();

			//sofa::helper::gl::LettersDL
					//	haptic_freq
			}

			Data<double> forceCoef;
			Data<double> momentCoef; 

			virtual void computeForce(double /*x*/, double /*y*/, double /*z*/, double /*u*/, double /*v*/, double /*w*/, double /*q*/, double& /*fx*/, double& /*fy*/, double& /*fz*/) {} ;
			void computeForce(const  VecCoord& state, const VecDeriv& /*Vstate*/,  VecDeriv& forces);
			virtual void computeWrench(const SolidTypes<double>::Transform &world_H_tool, const SolidTypes<double>::SpatialVector &V_tool_world, SolidTypes<double>::SpatialVector &W_tool_world );
			//void computeWrenchConstraint(const SolidTypes<double>::Transform &world_H_tool, const SolidTypes<double>::SpatialVector &V_tool_world, SolidTypes<double>::SpatialVector &W_tool_world );

			void setReferencePosition(SolidTypes<double>::Transform& referencePosition)
			{
				worldPosFreeSimu_buf = referencePosition;
			}

			VMechanismsForceFeedback();

			~VMechanismsForceFeedback()
			{
				delete(_timer);
			}



					/// Pre-construction check method called by ObjectFactory.
			/// Check that DataTypes matches the MechanicalState.
			template<class T>
			static bool canCreate(T*& obj, objectmodel::BaseContext* context, objectmodel::BaseObjectDescription* arg)
			{
				if (dynamic_cast<MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
					return false;
				return BaseObject::canCreate(obj, context, arg);
			}

			virtual std::string getTemplateName() const
			{
				return templateName(this);
			}

			static std::string templateName(const VMechanismsForceFeedback<DataTypes>* = NULL)
			{
				return DataTypes::Name();
			}

protected:
        core::behavior::MechanicalState<DataTypes> *mState; ///< The omni try to follow this mechanical state.
	
	// when using mastercontactsolver:
        //sofa::component::odesolver::LCP* lcp;
        //sofa::component::odesolver::MasterContactSolver* masterContactSolver;
        sofa::component::mastersolver::MasterContactSolver* masterContactSolver;
	
	// when using masterconstraintsolver:
        sofa::component::mastersolver::ConstraintProblem* cp;
        sofa::component::mastersolver::MasterConstraintSolver* masterConstraintSolver;

        //NEW
        bool mCurrBufferInUse;  // is current buffer in use right now
        std::vector<int> mBuferID[3];

        SolidTypes<double>::Transform worldPosFreeSimu_buf, worldPosFreeSimu;

        //\NEW
		VecCoord Xfree_ref, Xfree_buf;


	// timer: verifies the time rates of the haptic loop
	CTime *_timer;
	double time_buf;
	double haptic_freq;

	// compatibility with previous version (using mastercontactsolver)
	bool _old;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif
