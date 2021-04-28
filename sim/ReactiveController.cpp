#include "dart/collision/bullet/bullet.hpp"
#include "ReactiveController.h"
using namespace dart::dynamics;
namespace DPhy
{


void
ReactiveController::
SimStep()
{
	int num_body_nodes = mInterestedDof / 3;
	// mCharacter->GetSkeleton()->setSPDTarget(mPDTargetPositions, 600, 49);
	// mWorld->step(false);
	
	// torque limit
	Eigen::VectorXd torque = mCharacter->GetSPDForces(mPDTargetPositions, mPDTargetVelocities);
	for(int j = 0; j < num_body_nodes; j++) {
		int idx = mCharacter->GetSkeleton()->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->GetSkeleton()->getBodyNode(j)->getParentJoint()->getNumDofs();
		std::string name = mCharacter->GetSkeleton()->getBodyNode(j)->getName();
		double torquelim = mCharacter->GetTorqueLimit(name) * 1.5;
		double torque_norm = torque.block(idx, 0, dof, 1).norm();
		
		torque.block(idx, 0, dof, 1) = std::max(-torquelim, std::min(torquelim, torque_norm)) * torque.block(idx, 0, dof, 1).normalized();
	}

	// check contact of humanoid

	if (!dart::math::isNan(torque))
		mCharacter->GetSkeleton()->setForces(torque);

	mWorld->step(false);
	mTimeElapsed += 1;
}

// for bullet collision detector
bool ReactiveController::collide (const dart::dynamics::BodyNode* bn1,
	const dart::dynamics::BodyNode* bn2){
	bool collide = false;

	auto sf1 = bn1->getShapeNodes();
	auto sf2 = bn2->getShapeNodes();

	auto last_contact = this->mController->getLastContacts();

	for (auto s1: sf1){
		for (auto s2: sf2){
			auto contact = last_contact.find(make_pair(s1->getName(), s2->getName()));
			if (contact != last_contact.end() && contact->second.norm() > EPSILON) return true;
		}
	}

	return false;
}

void ReactiveController::
	UpdatePerceptionInfo()
{
	// check if character had contact
	// create a world with previous state without the contact, and simulate step to get result
	auto humanoid = this->mController->GetWorld()->getSkeleton("Humanoid");

	std::vector<dart::dynamics::ShapeNode *> humanoid_shapes;
	for(int i = 0; i < humanoid->getNumShapeNodes(); i++){
		humanoid_shapes.push_back(humanoid->getShapeNode(i));
	}

	bool body_contact = false;

	// check if body had contact with perturbances

	if (body_contact){
		auto sim_world = this->mController->GetWorld()->clone();
		for (auto skel : perturbance){
			sim_world->removeSkeleton(skel);
		}
		auto char_skel = sim_world->getSkeleton("Humanoid");
		char_skel->setVelocities(this->previous_velocities);
		char_skel->setPositions(this->previous_positions);

		// controller step != world step, fix this
		sim_world->step();

		d_expected_positions.push_back(this->mController->GetSkeleton()->getPositions() - char_skel->getPositions());
		d_expected_velocities.push_back(this->mController->GetSkeleton()->getVelocities() - char_skel->getVelocities());
	}
};

}