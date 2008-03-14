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
#ifndef SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_H
#define SOFA_COMPONENT_COLLISION_RAYPICKINTERACTOR_H 

#include <sofa/component/collision/RayModel.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/component/MechanicalObject.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/component/forcefield/StiffSpringForceField.h>
#include <sofa/component/forcefield/VectorSpringForceField.h>
//#include <sofa/component/constraint/LagrangianMultiplierAttachConstraint.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#ifdef SOFA_GPU_CUDA
#include <sofa/gpu/cuda/CudaMechanicalObject.h>
#include <sofa/gpu/cuda/CudaSpringForceField.h>
#endif


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;

class RayPickInteractor : public RayModel, public core::BehaviorModel
{
protected:
	typedef forcefield::VectorSpringForceField<Vec3Types> ContactForceField1;
//	typedef constraint::LagrangianMultiplierAttachConstraint<Vec3Types> ContactForceField2;
#ifdef SOFA_GPU_CUDA
        typedef sofa::gpu::cuda::CudaVec3Types CudaVec3Types;
	typedef forcefield::StiffSpringForceField<CudaVec3Types> CudaContactForceField1;
#endif
	sofa::helper::vector<core::componentmodel::behavior::BaseForceField*> forcefields;
	sofa::helper::vector<simulation::tree::GNode*> nodes;
	sofa::helper::vector< std::pair<int,double> > attachedPoints;
	component::MechanicalObject<Vec3Types>* mm;

	// Global variables to register the two last input points (for incision along one segment in a triangular mesh)
	Vec<3,double> a_init;
	Vec<3,double> b_init;
	unsigned int ind_ta_init;
	unsigned int ind_tb_init;

	bool is_first_cut;

	unsigned int b_last_init;
	sofa::helper::vector< unsigned int > b_p12_last_init;
	sofa::helper::vector< unsigned int > b_i123_last_init;

	unsigned int a_last_init;
	sofa::helper::vector< unsigned int >  a_p12_last_init;
	sofa::helper::vector< unsigned int >  a_i123_last_init;

#ifdef SOFA_GPU_CUDA
	sofa::helper::vector< std::pair<int,double> > cudaAttachedPoints;
	component::MechanicalObject<CudaVec3Types>* mmcuda;
#endif
        enum { DISABLED, PRESENT, ACTIVE, ATTACHED, FIRST_INPUT, SECOND_INPUT, IS_CUT } state; // FIRST_INPUT and SECOND_INPUT are states to control inputs points for incision in a triangular mesh
	int button; // index of activated button (only valid in ACTIVE state)
	Mat4x4d transform;
	core::componentmodel::collision::DetectionOutput* findFirstCollision(const Ray& ray, double* dist);
public:
	RayPickInteractor();
	~RayPickInteractor();
	
	virtual void init();
	
	virtual bool isActive();
	
	/// Computation of a new simulation step.
	virtual void updatePosition(double dt);
	
	/// Interactor interface
	virtual void newPosition(const Vector3& translation, const Quat& rotation, const Mat4x4d& transformation);
	virtual void newEvent(const std::string& command);
	
	virtual void draw();
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
