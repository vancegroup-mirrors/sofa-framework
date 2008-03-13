#ifndef SOFA_CONTRIB_TESTING_SENSABLE_H
#define SOFA_CONTRIB_TESTING_SENSABLE_H

#include <iostream>

#include "Sofa-old/Abstract/BehaviorModel.h"
#include "Sofa-old/Abstract/VisualModel.h"
#include "Sofa-old/Abstract/CollisionModel.h"
#include "Sofa-old/Components/SphereModel.h"
#include "Sofa-old/Components/Common/RigidTypes.h"

using std::cout;
using namespace Sofa::Abstract;
using Sofa::Components::SphereModel;
using Sofa::Core::MechanicalModel;
using Sofa::Components::Common::RigidTypes;
using Sofa::Components::Common::Quat;
using Sofa::Components::Common::Vec3d;
using Sofa::Components::Common::Mat3x3d;

class SensAble : public VisualModel, public BehaviorModel
{

public:

	SensAble();
	~SensAble();

	// -- VisualModel interface
	void draw();
	void initTextures() { }
	void update() { }

	double tipOffset;
	Vec3d translation;
	Quat rotation;
	Mat3x3d mrotation;
	double scale;

	Vec3d pivot;

	SphereModel* sphereModel;
	MechanicalModel<RigidTypes>* rigidModel;

	// -- CollisionModel interface
/*
	virtual std::vector<CollisionElement*> & getCollisionElements();
	virtual CollisionModel* getNext() { return sphereModel->getNext(); }
	virtual CollisionModel* getPrevious() { return sphereModel->getPrevious(); }
	virtual BehaviorModel* getObject() { return sphereModel->getObject(); }
*/
	// -- BehaviorModel interface (these would conflict with DynamicModel!)
	virtual void init();

	/// Computation of a new simulation step.
	virtual void updatePosition(double dt);

	Sofa::Components::Common::Vector3 currentForce;
};

#endif