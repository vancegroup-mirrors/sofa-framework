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
#include <sofa/simulation/tree/MechanicalKaapiAction.h>
#include <sofa/simulation/tree/GNode.h>
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa
{

namespace simulation
{

namespace tree
{

Action::Result MechanicalKaapiAction::processNodeTopDown(GNode* node)
{
	Result res = RESULT_CONTINUE;
	if (node->mechanicalState != NULL) {
		if (node->mechanicalMapping != NULL) {
			//cerr<<"MechanicalKaapiAction::processNodeTopDown, node "<<node->getName()<<" is a mapped model"<<endl;
			Result res2;
			ctime_t t0 = begin(node, node->mechanicalMapping);
			res = this->fwdMechanicalMapping(node, node->mechanicalMapping);
			end(node, node->mechanicalMapping, t0);
			t0 = begin(node, node->mechanicalState);
			res2 = this->fwdMappedMechanicalState(node, node->mechanicalState);
			end(node, node->mechanicalState, t0);
			if (res2 == RESULT_PRUNE)
				res = res2;
		} else {
			//cerr<<"MechanicalKaapiAction::processNodeTopDown, node "<<node->getName()<<" is a no-map model"<<endl;
			ctime_t t0 = begin(node, node->mechanicalState);
			res = this->fwdMechanicalState(node, node->mechanicalState);
			end(node, node->mechanicalState, t0);
		}
	}
	if (res != RESULT_PRUNE) {
		if (node->mass != NULL) {
			ctime_t t0 = begin(node, node->mass);
			res = this->fwdMass(node, node->mass);
			end(node, node->mass, t0);
		}
	}
	if (res != RESULT_PRUNE) {
		res = for_each_r(this, node, node->forceField, &MechanicalKaapiAction::fwdForceField);
	}
        if (res != RESULT_PRUNE) {
            res = for_each_r(this, node, node->interactionForceField, &MechanicalKaapiAction::fwdInteractionForceField);
        }
        if (res != RESULT_PRUNE) {
            res = for_each_r(this, node, node->constraint, &MechanicalKaapiAction::fwdConstraint);
        }
        return res;
}

void MechanicalKaapiAction::processNodeBottomUp(GNode* node)
{
	if (node->mechanicalState != NULL) {
		if (node->mechanicalMapping != NULL) {
			ctime_t t0 = begin(node, node->mechanicalState);
			this->bwdMappedMechanicalState(node, node->mechanicalState);
			end(node, node->mechanicalState, t0);
			t0 = begin(node, node->mechanicalMapping);
			this->bwdMechanicalMapping(node, node->mechanicalMapping);
			end(node, node->mechanicalMapping, t0);
		} else {
			ctime_t t0 = begin(node, node->mechanicalState);
			this->bwdMechanicalState(node, node->mechanicalState);
			end(node, node->mechanicalState, t0);
		}
	}
}

MechanicalKaapiPropagatePositionAndVelocityAction::MechanicalKaapiPropagatePositionAndVelocityAction(double t, VecId x, VecId v) : t(t), x(x), v(v)
{
    //cerr<<"::MechanicalPropagatePositionAndVelocityAction"<<endl;
}


Action::Result MechanicalKaapiPropagatePositionAndVelocityAction::processNodeTopDown(GNode* node)
{
	//cerr<<" MechanicalPropagatePositionAndVelocityAction::processNodeTopDown "<<node->getName()<<endl;   
	node->setTime(t);
        node->updateContext();
        return MechanicalKaapiAction::processNodeTopDown( node);
}

void MechanicalKaapiPropagatePositionAndVelocityAction::processNodeBottomUp(GNode* node)
{
	//cerr<<" MechanicalKaapiPropagatePositionAndVelocityAction::processNodeBottomUp "<<node->getName()<<endl;   
	for_each(this, node, node->constraint, &MechanicalKaapiPropagatePositionAndVelocityAction::bwdConstraint);

}



// void MechanicalKaapiVOpAction::processNodeBottomUp(GNode* node)
// {
//    MechanicalKaapiAction::processNodeBottomUp(node);
//    if( node->getDebug() && node->mechanicalModel ) {
//       cerr<<"MechanicalKaapiVopAction::processNodeBottomUp, node "<<node->getName()<<", x=";
//       (*node->mechanicalModel).printX(cerr);
//       cerr<<endl;
//       cerr<<"MechanicalKaapiVopAction::processNodeBottomUp, node "<<node->getName()<<", v=";
//       (*node->mechanicalModel).printV(cerr);
//       cerr<<endl;
//       cerr<<"MechanicalKaapiVopAction::processNodeBottomUp, node "<<node->getName()<<", f=";
//       (*node->mechanicalModel).printF(cerr);
//       cerr<<endl;
//       cerr<<"MechanicalKaapiVopAction::processNodeBottomUp, node "<<node->getName()<<", dx=";
//       (*node->mechanicalModel).printDx(cerr);
//       cerr<<endl;
//    }
// }

} // namespace tree

} // namespace simulation

} // namespace sofa

