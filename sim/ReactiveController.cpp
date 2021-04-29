#include "dart/collision/bullet/bullet.hpp"
#include "ReactiveController.h"
using namespace dart::dynamics;
namespace DPhy
{

ReactiveController::
ReactiveController(ReferenceManager* ref, const std::string character_path, bool record, int id): Controller(ref, character_path, record, id)
{
	this->mVirtualWorld =  this->mWorld->clone();
	this->mVirtualWorld->removeSkeleton(this->mVirtualWorld->getSkeleton(this->GetSkeleton()->getName()));
	this->mVirtualCharacter = new DPhy::Character(character_path);
	this->mVirtualWorld->addSkeleton(this->mCharacter->GetSkeleton());
	this->mVirtualCharacter->SetPDParameters(600, 49);

	auto collisionEngine = mVirtualWorld->getConstraintSolver()->getCollisionDetector();
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("LeftFoot"));
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("RightFoot"));
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("LeftToe"));
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("RightToe"));
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("LeftHand"));
	collisionEngine->createCollisionGroup(this->mVirtualCharacter->GetSkeleton()->getBodyNode("RightHand"));
	collisionEngine->createCollisionGroup(this->mVirtualWorld->getSkeleton("Ground").get());
}

// Eigen::VectorXd 
// ReactiveController::
// GetEndEffectorStatePosAndVel(const Eigen::VectorXd pos, const Eigen::VectorXd vel) {
// 	Eigen::VectorXd ret;
// 	auto& skel = mVirtualCharacter->GetSkeleton();
// 	BodyNode* root = skel->getRootBodyNode();
// 	Eigen::Isometry3d cur_root_inv = root->getWorldTransform().inverse();

// 	int num_ee = mEndEffectors.size();
// 	Eigen::VectorXd p_save = skel->getPositions();
// 	Eigen::VectorXd v_save = skel->getVelocities();

// 	skel->setPositions(pos);
// 	skel->setVelocities(vel);
// 	skel->computeForwardKinematics(true, true, false);

// 	ret.resize((num_ee)*12+15);
// //	ret.resize((num_ee)*9+12);

// 	for(int i=0;i<num_ee;i++)
// 	{		
// 		Eigen::Isometry3d transform = cur_root_inv * skel->getBodyNode(mEndEffectors[i])->getWorldTransform();
// 		//Eigen::Quaterniond q(transform.linear());
// 		// Eigen::Vector3d rot = QuaternionToDARTPosition(Eigen::Quaterniond(transform.linear()));
// 		ret.segment<9>(9*i) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
// 							   transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2), 
// 							   transform.translation();
// //		ret.segment<6>(6*i) << rot, transform.translation();
// 	}


// 	for(int i=0;i<num_ee;i++)
// 	{
// 	    int idx = skel->getBodyNode(mEndEffectors[i])->getParentJoint()->getIndexInSkeleton(0);
// 		ret.segment<3>(9*num_ee + 3*i) << vel.segment<3>(idx);
// //	    ret.segment<3>(6*num_ee + 3*i) << vel.segment<3>(idx);

// 	}

// 	// root diff with target com
// 	Eigen::Isometry3d transform = cur_root_inv * skel->getRootBodyNode()->getWorldTransform();
// 	//Eigen::Quaterniond q(transform.linear());

// 	Eigen::Vector3d rot = QuaternionToDARTPosition(Eigen::Quaterniond(transform.linear()));
// 	Eigen::Vector3d root_angular_vel_relative = cur_root_inv.linear() * skel->getRootBodyNode()->getAngularVelocity();
// 	Eigen::Vector3d root_linear_vel_relative = cur_root_inv.linear() * skel->getRootBodyNode()->getCOMLinearVelocity();

// 	ret.tail<15>() << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
// 					  transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2),
// 					  transform.translation(), root_angular_vel_relative, root_linear_vel_relative;
// //	ret.tail<12>() << rot, transform.translation(), root_angular_vel_relative, root_linear_vel_relative;

// 	// restore
// 	skel->setPositions(p_save);
// 	skel->setVelocities(v_save);
// 	//skel->computeForwardKinematics(true, true, false);

// 	return ret;
// }

// Eigen::VectorXd
// ReactiveController::
// GetState()
// {
// 	// State Component : Joint_angle, Joint_velocity, up_vector_angle, p_next,
// 	if(mIsTerminal && terminationReason != 8){
// 		return Eigen::VectorXd::Zero(mNumState);
// 	}
// 	SkeletonPtr skel = mVirtualCharacter->GetSkeleton();
	
// 	double root_height = skel->getRootBodyNode()->getCOM()[1];

// 	Eigen::VectorXd p_save = skel->getPositions();
// 	Eigen::VectorXd v_save = skel->getVelocities();
// 	Eigen::VectorXd p,v;
// 	// p.resize(p_save.rows()-6);
// 	// p = p_save.tail(p_save.rows()-6);

// 	int n_bnodes = mVirtualCharacter->GetSkeleton()->getNumBodyNodes();
// 	int num_p = (n_bnodes - 1) * 6;
// 	p.resize(num_p);

// 	for(int i = 1; i < n_bnodes; i++){
// 		Eigen::Isometry3d transform = skel->getBodyNode(i)->getRelativeTransform();
// 		// Eigen::Quaterniond q(transform.linear());
// 		//	ret.segment<6>(6*i) << rot, transform.translation();
// 		p.segment<6>(6*(i-1)) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
// 								 transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2);
// 	}

// 	v = v_save;

// 	BodyNode* root = skel->getRootBodyNode();
// 	Eigen::Isometry3d cur_root_inv = root->getWorldTransform().inverse();

// 	Eigen::Vector3d up_vec = root->getTransform().linear()*Eigen::Vector3d::UnitY();
// 	double up_vec_angle = atan2(std::sqrt(up_vec[0]*up_vec[0]+up_vec[2]*up_vec[2]),up_vec[1]);

// 	// The angles and velocity of end effector & root info
// 	Eigen::VectorXd ee;
// 	ee.resize(mEndEffectors.size()*3);
// 	for(int i=0;i<mEndEffectors.size();i++)
// 	{
// 		Eigen::Isometry3d transform = cur_root_inv * skel->getBodyNode(mEndEffectors[i])->getWorldTransform();
// 		ee.segment<3>(3*i) << transform.translation();
// 	}
// 	double t = mReferenceManager->GetTimeStep(mCurrentFrameOnPhase);

// 	Motion* p_v_target = mReferenceManager->GetMotion(mCurrentFrame+t);
// 	Eigen::VectorXd p_now = p_v_target->GetPosition();
// 	// The rotation and translation of end effector in the future(future 1 frame)
// 	Eigen::VectorXd p_next = GetEndEffectorStatePosAndVel(p_now, p_v_target->GetVelocity()*t);
	
// 	delete p_v_target;

// 	double phase = ((int) mCurrentFrame % mReferenceManager->GetPhaseLength()) / (double) mReferenceManager->GetPhaseLength();
// 	Eigen::VectorXd state;

// 	double com_diff = 0;
	
// 	state.resize(p.rows()+v.rows()+1+1+p_next.rows()+ee.rows()+2);
// 	state<< p, v, up_vec_angle, root_height, p_next, mAdaptiveStep, ee, mCurrentFrameOnPhase;

// 	return state;
// }

// void
// ReactiveController::
// SetPDTarget(){
// 	int num_body_nodes = mInterestedDof / 3;
// 	int dof = this->mVirtualCharacter->GetSkeleton()->getNumDofs(); 

// 	for(int i = 0; i < mInterestedDof; i++){
// 		mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
// 	}
// 	int sign = 1;
// 	if(mActions[mInterestedDof] < 0)
// 		sign = -1;
	
// 	mActions[mInterestedDof] = dart::math::clip(mActions[mInterestedDof]*1.2, -2.0, 1.0);
// 	mActions[mInterestedDof] = exp(mActions[mInterestedDof]);

// 	this->mPrevFrameOnPhase = this->mCurrentFrameOnPhase;
// 	this->mCurrentFrame+=1;
// 	this->mCurrentFrameOnPhase+=1;
// 	nTotalSteps += 1;
// 	int n_bnodes = mVirtualCharacter->GetSkeleton()->getNumBodyNodes();

// 	Motion* p_v_target = mReferenceManager->GetMotion(mCurrentFrame);
// 	Eigen::VectorXd p_now = p_v_target->GetPosition();
// 	this->mTargetPositions = p_now ; //p_v_target->GetPosition();
// 	this->mTargetVelocities = mVirtualCharacter->GetSkeleton()->getPositionDifferences(mTargetPositions, mPrevTargetPositions) / 0.033;
// 	delete p_v_target;

// 	p_v_target = mReferenceManager->GetMotion(mCurrentFrame);
// 	this->mPDTargetPositions = p_v_target->GetPosition();
// 	this->mPDTargetVelocities = p_v_target->GetVelocity();
// 	delete p_v_target;


// 	int count_dof = 0;

// 	for(int i = 1; i <= num_body_nodes; i++){
// 		int idx = mVirtualCharacter->GetSkeleton()->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
// 		int dof = mVirtualCharacter->GetSkeleton()->getBodyNode(i)->getParentJoint()->getNumDofs();
// 		mPDTargetPositions.block(idx, 0, dof, 1) += mActions.block(count_dof, 0, dof, 1);
// 		count_dof += dof;
// 	}
// }

// void
// ReactiveController::
// SimStep()
// {
// 	int num_body_nodes = mInterestedDof / 3;

// 	// torque limit
// 	Eigen::VectorXd torque = mVirtualCharacter->GetSPDForces(mPDTargetPositions, mPDTargetVelocities);

// 	for(int j = 0; j < num_body_nodes; j++) {
// 		int idx = mVirtualCharacter->GetSkeleton()->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
// 		int dof = mVirtualCharacter->GetSkeleton()->getBodyNode(j)->getParentJoint()->getNumDofs();
// 		std::string name = mCharacter->GetSkeleton()->getBodyNode(j)->getName();
// 		double torquelim = mCharacter->GetTorqueLimit(name) * 1.5;
// 		double torque_norm = torque.block(idx, 0, dof, 1).norm();
		
// 		torque.block(idx, 0, dof, 1) = std::max(-torquelim, std::min(torquelim, torque_norm)) * torque.block(idx, 0, dof, 1).normalized();
// 	}

// 	// check contact of humanoid
// 	bool body_contact = this->HumanoidCollide();

// 	if (dart::math::isNan(torque))
// 		torque.setZero();

// 	mCharacter->GetSkeleton()->setForces(torque);

// 	if (body_contact){
// 		auto char_skel = mVirtualCharacter->GetSkeleton();
// 		char_skel->setVelocities(this->GetSkeleton()->getVelocities());
// 		char_skel->setPositions(this->GetSkeleton()->getPositions());

// 		mVirtualCharacter->GetSkeleton()->setForces(torque);

// 		mVirtualWorld->step();
// 		mWorld->step(false);

// 		d_expected_positions.push_back(char_skel->getPositions() - this->GetSkeleton()->getPositions());
// 		d_expected_velocities.push_back(char_skel->getVelocities() - this->GetSkeleton()->getVelocities());
// 		contact_timestamp.push_back(this->mTimeElapsed);
// 	}
// 	else {
// 		mWorld->step(false);
// 	}
// 	UpdatePerceptionInfo();
// 	mTimeElapsed += 1;

// 	auto collisionSolver = mWorld->getConstraintSolver();
// 	this->mLastCollision = collisionSolver->getLastCollisionResult();
// }

std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, Controller::pair_hash> ReactiveController::getLastContacts(){
	std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, Controller::pair_hash> contacts;

	for (auto contact: this->mLastCollision.getContacts()){
		// get contacts at the end of step and write collision info somewhere
		auto co1 = contact.collisionObject1;
		auto co2 = contact.collisionObject2;

		auto sf1 = co1->getShapeFrame()->getName();
		auto sf2 = co2->getShapeFrame()->getName();

		contacts.insert(make_pair(make_pair(sf1, sf2), contact.force));
		// contact check could be done for both side instead?
		// this->mLastContacts.insert(make_pair(make_pair(sf2, sf1), -contact.force));
	}

	return contacts;
}

// for bullet collision detector
bool ReactiveController::HumanoidCollide (){
	std::vector<dart::dynamics::ShapeNode*> humanoidNodes;
	auto bds = this->mVirtualCharacter->GetSkeleton()->getBodyNodes();
	for (auto bd : bds){
		auto shapes = bd->getShapeNodes();
		humanoidNodes.insert(humanoidNodes.end(), shapes.begin(), shapes.end());
	}

	std::vector<dart::dynamics::ShapeNode*> perturbanceNodes;

	for (auto p : this->perturbance){
		auto p_bds = p->getBodyNodes();
		for (auto bd : p_bds){
			auto shapes = bd->getShapeNodes();
			perturbanceNodes.insert(perturbanceNodes.end(), shapes.begin(), shapes.end());
		}
	}

	auto last_contact = this->getLastContacts();

	for (auto s1 = humanoidNodes.begin(); s1 < humanoidNodes.end(); s1++){
		for (auto s2 = perturbanceNodes.begin(); s1 < perturbanceNodes.end(); s1++){
			auto contact = last_contact.find(std::make_pair((*s1)->getName(), (*s2)->getName()));
			if (contact != last_contact.end() && contact->second.norm() > EPSILON) return true;
		}
	}

	return false;
}

void ReactiveController::
	UpdatePerceptionInfo()
{
	// perceived state = actual state + delta (decays over time)
	auto skel = mCharacter->GetSkeleton();
	auto vSkel = mVirtualCharacter->GetSkeleton();
	Eigen::VectorXd positions = skel->getPositions();
	Eigen::VectorXd velocities = skel->getVelocities();

	while(!contact_timestamp.empty() && contact_timestamp.front() < this->mTimeElapsed - (mSimulationHz / 4)){
		contact_timestamp.erase(contact_timestamp.begin());
		d_expected_positions.erase(d_expected_positions.begin());
		d_expected_velocities.erase(d_expected_velocities.begin());
	}

	for (int i = 0; i < contact_timestamp.size(); i++){
		int contactTimeElapsed = this->mTimeElapsed - contact_timestamp[i];
		double weight = 1.0 - (exp(contactTimeElapsed) - 1) / (exp(mSimulationHz / 4) - 1);
		auto d_v = d_expected_velocities[i] * weight;
		velocities += d_v;
		positions += d_expected_positions[i] * weight + (1.0 / mSimulationHz) * d_v * contactTimeElapsed;
	}

	vSkel->setPositions(positions);
	vSkel->setVelocities(velocities);
}

void 
ReactiveController::
ClearRecord() 
{
	Controller::ClearRecord();
	this->mRecordVirtualVelocity.clear();
	this->mRecordVirtualPosition.clear();

	// virtual character should be updated before recording in controller reset
	this->mVirtualWorld->reset();
	auto vskel = this->mVirtualCharacter->GetSkeleton();
	vskel->clearConstraintImpulses();
	vskel->clearInternalForces();
	vskel->clearExternalForces();
	vskel->setPositions(this->GetSkeleton()->getPositions());
	vskel->setVelocities(this->GetSkeleton()->getVelocities());
	d_expected_positions.clear();
	d_expected_velocities.clear();
	contact_timestamp.clear();
}

void
ReactiveController::
SaveStepInfo() 
{
	Controller::SaveStepInfo();
	mRecordVirtualPosition.push_back(mVirtualCharacter->GetSkeleton()->getPositions());
	mRecordVirtualVelocity.push_back(mVirtualCharacter->GetSkeleton()->getVelocities());
}

void
ReactiveController::
Reset(bool RSI)
{
	Controller::Reset(RSI);
}

}