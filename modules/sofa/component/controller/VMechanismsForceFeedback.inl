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
#include <sofa/component/controller/VMechanismsForceFeedback.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/mastersolver/MasterContactSolver.h>
#include <sofa/component/mastersolver/MasterConstraintSolver.h>
#include <sofa/helper/LCPcalc.h>

using namespace std;
using namespace sofa::defaulttype;

namespace sofa
{
    namespace component
    {
        namespace controller
        {


            template <class DataTypes>
                    VMechanismsForceFeedback<DataTypes>::VMechanismsForceFeedback()
                        : forceCoef(initData(&forceCoef, 0.03, "forceCoef","multiply haptic force by this coef.")),
                        momentCoef(initData(&momentCoef, 0.0, "momentCoef","multiply haptic moment by this coef (for 6D rendering)")),
                        masterContactSolver(NULL),
                        masterConstraintSolver(NULL),
                        haptic_freq(0.0)
            {
                _timer = new CTime();
                _old = false;
            }

            template <class DataTypes>
                    void VMechanismsForceFeedback<DataTypes>::init()
            {
				std::cout<<" begin init"<<std::endl;
                core::objectmodel::BaseContext* context = this->getContext();

                this->ForceFeedback::init();

                //	BaseObject* object2 = static_cast<BaseObject*>(context->getObject(classid(sofa::component::odesolver::MasterContactSolver)));
                if (!context) {
                    serr << "VMechanismsForceFeedback has no current context. Initialisation failed." << sendl;
                    return;
                }

                context->get(masterConstraintSolver);                
                //masterContactSolver = context->get<sofa::component::odesolver::MasterContactSolver>();
                //masterConstraintSolver = context->get<sofa::component::mastersolver::MasterConstraintSolver>();
                mState = dynamic_cast<core::behavior::MechanicalState<DataTypes> *> (context->getMechanicalState());

                if (!mState)
                    serr << "NLCPForceFeedback has no binding MechanicalState" << sendl;

                if (masterContactSolver!=NULL)
                {
                    //lcp = masterContactSolver->getLCP();
                    _old = true;
                }
                else if(masterConstraintSolver !=NULL)
                {
                    // verify that masterConstraintSolver is using a double buffer:
                    if(!masterConstraintSolver->doubleBuffer.getValue())
                    {
			serr<<"masterConstraintSolver must use a double Buffer in order to use NLCPForceFeedback"<<sendl;
			masterConstraintSolver->doubleBuffer.setValue(true);
                    }
                    cp = masterConstraintSolver->getConstraintProblem();
                }
                else
                {
                    serr << "NLCPForceFeedback has not found any MasterSolver" << sendl;
                    f_activate.setValue(false);
                }


                sout << "init NLCPForceFeedback done " << sendl;
            };

 template <class DataTypes>
     void VMechanismsForceFeedback<DataTypes>::computeForce(const VecCoord& state, const VecDeriv &/*velocity*/, VecDeriv& forces)
            {
				
                if (!f_activate.getValue()) {
                    return;
                }

                double actualTime = (double)_timer->getTime()/ (double)CTime::getTicksPerSec();
                haptic_freq = 1/ (  actualTime - time_buf) ;
                time_buf = actualTime;

                if (!masterConstraintSolver || !mState)   //??? already tested in init
                {
                    // error message using std::cerr (and not serr) because of multithreaded callback
                    std::cerr<<"pb in NLCPForceFeedback : no masterConstraintSolver or no mStates found"<<std::endl;
                    return;
                }

                static component::mastersolver::ConstraintProblem* cp_buf = NULL;
                // buffer the Xfree position (each time computeWrenchConstraint is called)
                static VecCoord Xfree_buf;
                // when the cp changes in the mastersolver, use Xfree_buf as reference for buffer cp
                static VecCoord Xfree_ref;

                mCurrBufferInUse = true;

                //static RigidTypes::VecConst c_buf;
				// buffer the matrix of the constraints
                static MatrixDeriv c_buf;
				// when the cp chantes in the mastersolver, use c_buf as reference for buffer cp
				static MatrixDeriv c_ref;

                ///////////////////// INIT : here the first time the function is called:
                if (cp_buf == NULL)
                {
					std::cout<<" INIT: first time the function is called !"<<std::endl;
                    cp_buf = masterConstraintSolver->getConstraintProblem();
					// copy of Xfree...
					Xfree_ref.resize( mState->getXfree()->size() );
					Xfree_buf.resize( mState->getXfree()->size() );
					for (unsigned int i=0; i<mState->getXfree()->size(); i++)
					{
						Xfree_ref[i] = (*mState->getXfree())[i];
						Xfree_buf[i] = (*mState->getXfree())[i];
					}

                }

                cp = masterConstraintSolver->getConstraintProblem();


                //////////////////// NEW CP : MasterConstraintSolver has computed a new constraint problem and has found a solution for it
                if (cp_buf!=cp)
                {
                    //std::cout<<"new Constraint Problem detected"<<std::endl;

					//  obtain the new reference position (the "free" position of the device used in the simulation)
                    Xfree_ref = Xfree_buf;
					c_ref=c_buf;

					const MatrixDeriv& constraints = c_ref;

                    //DEBUG !! verify the constraints that correspond to the lcp in vecConst c
                    MatrixDerivRowConstIterator rowItEnd = constraints.end();
                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
						if(rowIt.index() >= (cp) -> getDfree()->size())
							std::cout<<"PROBLEM: rIndex="<<rowIt.index()<<"  - dfree.size="<<(cp) -> getDfree()->size()<<std::endl;
                    }

					// the new problem is copy in a buffer and can be "re-computed" in the haptic simulation 

                }


                // the used constraint problem is put in the buffer  (to be able to do the comparison between new cp and previous one)
                cp_buf = cp;



                const MatrixDeriv& constraints = c_ref;


                if(cp)
                {
					VecDeriv dx;

                    //is this OK or "state" (the actual position of the tool) should be converted to simulation frame?
					if(state.size() <= Xfree_ref.size())
					{
						dx.resize(state.size());
						for (unsigned int i=0; i<state.size(); i++)
						{
							dx[i] = state[i] - Xfree_ref[i];
						}
					}
					else
					{
						// problem: more degrees of Freedom for the interface than for the MState....
						cerr<<"in computeForce state.size() > Xfree.size()"<<std::endl;
					}



					// modify the right-end term (the violation) of the constraint problem based on the new state of the device
                    MatrixDerivRowConstIterator rowItEnd = constraints.end();
                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                        MatrixDerivColConstIterator colItEnd = rowIt.end();
                        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                        {
                            double dDelta = dot(colIt.val(), dx[colIt.index()]);
                            (*(cp) -> getDfree())[rowIt.index()] += dDelta;
                        }
                    }


					// recompute the hole constraint problem
                    double timeOut = 0.0008;  //0.8ms  // TODO => put  in a data !
                    (cp)->gaussSeidelConstraintTimed(timeOut, 100);

					// reset the right-end term (the violation) for the ref position in the simulation
                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                        MatrixDerivColConstIterator colItEnd = rowIt.end();
                        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                        {
                            double dDelta = dot(colIt.val(), dx[colIt.index()]);
                            (*(cp) -> getDfree())[rowIt.index()] -= dDelta;
                        }
                    }


                    //////// Compute the correspondant force to apply it on the device
                    forces.clear();
                    forces.resize(Xfree_buf.size());

                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                            if ( (*(cp)->getF()) [rowIt.index()] != 0.0)
                            {
                                    MatrixDerivColConstIterator colItEnd = rowIt.end();
                                    for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                                    {
                                            forces[colIt.index()] += colIt.val() *  (*(cp)->getF())[rowIt.index()];
                                    }
                            }
                    }
                } //if cp


               // the actual free position is put in the buffer
				// copy of Xfree...
				Xfree_buf.resize( mState->getXfree()->size() );
				for (unsigned int i=0; i<mState->getXfree()->size(); i++)
				{
					Xfree_buf[i] = (*mState->getXfree())[i];
				}

                // copy of the constraints that correspond to the lcp in vecConst c
                c_buf.clear();
                const MatrixDeriv& c = *(mState->getC());
                MatrixDerivRowConstIterator rowItEnd = c.end();
                for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
                {
                    c_buf.addLine(rowIt.index(), rowIt.row());
                }


            };

            template <class DataTypes>
                    void VMechanismsForceFeedback<DataTypes>::computeWrench(const SolidTypes<double>::Transform &/*world_H_tool*/,
                                                                            const SolidTypes<double>::SpatialVector &/*V_tool_world*/,
                                                                            SolidTypes<double>::SpatialVector &/*W_tool_world*/ )
            {
                //compute force should be used for Vec types!
            };



            // 6D rendering of contacts
            using sofa::defaulttype::Rigid3dTypes;

            template <>
                    void VMechanismsForceFeedback<Rigid3dTypes>::computeWrench(const SolidTypes<double>::Transform &world_H_tool,
                                                                                      const SolidTypes<double>::SpatialVector &/*V_tool_world*/,
                                                                                      SolidTypes<double>::SpatialVector &W_tool_world )
            {
				std::cout<<" begin 2"<<std::endl;

                if (!f_activate.getValue()) {
                    return;
                }

                double actualTime = (double)_timer->getTime()/ (double)CTime::getTicksPerSec();
                haptic_freq = 1/ (  actualTime - time_buf) ;
                time_buf = actualTime;

                if (!f_activate.getValue())
                {
                    return;
                }

                if (!masterConstraintSolver || !mState)   //??? already tested in init
                {
                    // error message using std::cerr (and not serr) because of multithreaded callback
                    std::cerr<<"pb in NLCPForceFeedback : no masterConstraintSolver found"<<std::endl;
                    return;
                }               

                static component::mastersolver::ConstraintProblem* cp_buf = NULL;
                // buffer the Xfree position (each time computeWrench is called)
                static Coord Xfree_buf;
                // when the cp changes in the mastersolver, use Xfree_buf as reference for buffer cp
                static Coord Xfree_ref;


                mCurrBufferInUse = true;

                //static RigidTypes::VecConst c_buf;
				// buffer for the matrix of the constraints
                static MatrixDeriv cf_buf;


                ///////////////////// INIT : here the first time the function is called:
                if (cp_buf == NULL)
                {
                    cp_buf = masterConstraintSolver->getConstraintProblem();
                    Xfree_ref = (*mState->getXfree())[0];
                    Xfree_buf = (*mState->getXfree())[0];
                }

                cp = masterConstraintSolver->getConstraintProblem();

                //////////////////// NEW CP : MasterConstraintSolver has computed a new constraint problem and has found a solution for it
                if (cp_buf!=cp)
                {
                    //std::cout<<"new LCP detected"<<std::endl;
                    Xfree_ref = Xfree_buf; // X ? ? ?
                    worldPosFreeSimu = worldPosFreeSimu_buf;

                    // copy of the constraints that correspond to the lcp in vecConst c
                    cf_buf.clear();

                    const MatrixDeriv& c = *(mState->getC());

                    MatrixDerivRowConstIterator rowItEnd = c.end();

                    for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                        cf_buf.addLine(rowIt.index(), rowIt.row());
                    }

                }
                // the used constraint problem is put in the buffer
                cp_buf = cp;

                // the actual free position is put in the buffer
                Xfree_buf = (*mState->getXfree())[0];

                const MatrixDeriv& constraints = cf_buf;

                if(cp)
                {                
                    //const unsigned int numConstraints = constraints->size();

                    SolidTypes<double>::Transform world_H_PosFreeSimu = SolidTypes<double>::Transform(Xfree_ref.getCenter(),Xfree_ref.getOrientation());
                    //SolidTypes<double>::Transform world_H_PosFreeSimu = worldPosFreeSimu;   //set in the step of the simulation
                    SolidTypes<double>::Transform PosFreeSimu_H_tool = world_H_PosFreeSimu.inversed() * world_H_tool;

                    SolidTypes<double>::SpatialVector DX = PosFreeSimu_H_tool.DTrans();
                    SolidTypes<double>::SpatialVector DX_world;
                    DX_world.setLinearVelocity(world_H_PosFreeSimu.projectVector(DX.getLinearVelocity()));
                    DX_world.setAngularVelocity(world_H_PosFreeSimu.projectVector(DX.getAngularVelocity()));

                    //std::vector<double> Ddelta;
                    //Ddelta.resize(cp->getSize());

                    MatrixDerivRowConstIterator rowItEnd = constraints.end();

                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                        MatrixDerivColConstIterator colItEnd = rowIt.end();
                        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                        {
                            double dDelta = colIt.val().getVCenter() * DX_world.getLinearVelocity()
                                            + colIt.val().getVOrientation() * DX_world.getAngularVelocity();
                            (*(cp) -> getDfree())[rowIt.index()] += dDelta;
                        }
                    }


                    double timeOut = 0.0008;  //0.8ms
                    (cp)->gaussSeidelConstraintTimed(timeOut, 100);

                    //std::cout<<"gs ended"<<std::endl;                    
                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                        MatrixDerivColConstIterator colItEnd = rowIt.end();
                        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                        {
                            double dDelta = colIt.val().getVCenter() * DX_world.getLinearVelocity()
                                            + colIt.val().getVOrientation() * DX_world.getAngularVelocity();
                            (*(cp) -> getDfree())[rowIt.index()] -= dDelta;
                        }
                    }                    

                    //////// Compute the correspondant force to apply it on the device
                    VecDeriv force;
                    force.clear();
                    force.resize((*mState->getX()).size());

                    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
                    {
                            if ( (*(cp)->getF()) [rowIt.index()] != 0.0)
                            {
                                    MatrixDerivColConstIterator colItEnd = rowIt.end();
                                    for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                                    {
                                            force[0] += colIt.val() *  (*(cp)->getF())[rowIt.index()];
                                    }
                            }
                    }
                    W_tool_world.setForce( force[0].getVCenter()    * forceCoef.getValue() );
                    W_tool_world.setTorque(force[0].getVOrientation() * momentCoef.getValue());

                }
				std::cout<<" end 2"<<std::endl;


            };



            template <>
                    void VMechanismsForceFeedback<Rigid3dTypes>::computeForce(const VecCoord& /*state*/, const VecDeriv& /*velocity*/, VecDeriv& /*forces*/)
            {
                //computeWrenche should be used for Vec types
            };

        } // namespace controller
    } // namespace component
} // namespace sofa
