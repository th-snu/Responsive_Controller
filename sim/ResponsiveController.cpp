#include "dart/collision/bullet/bullet.hpp"
#include "ResponsiveController.h"
using namespace dart::dynamics;
using namespace std;
namespace DPhy
{

ResponsiveController::
ResponsiveController(ReferenceManager* ref, const std::string character_path, bool record, int id): Controller(ref, character_path, record, id)
{
	this->recovery_mode = false;
	this->mVirtualWorld = std::make_shared<dart::simulation::World>();
	this->mVirtualWorld->setTimeStep(1.0/(double)mSimulationHz);
	this->mVirtualWorld->setGravity(Eigen::Vector3d(0,-9.8,0));	
	this->mVirtualWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(mVirtualWorld->getConstraintSolver())->setBoxedLcpSolver(std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
	auto vGround = DPhy::SkeletonBuilder::BuildFromFile(std::string(PROJECT_DIR)+std::string("/character/ground.xml")).first;
	vGround->getBodyNode(0)->setFrictionCoeff(1.0);
	this->mVirtualWorld->addSkeleton(vGround);

	this->mVirtualCharacter = new DPhy::Character(character_path);
	this->mVirtualWorld->addSkeleton(this->mVirtualCharacter->GetSkeleton());
	
	this->mVirtualCharacter->SetPDParameters(600, 49);

	// this->GetSkeleton()->disableSelfCollisionCheck();
	// this->mVirtualCharacter->GetSkeleton()->disableSelfCollisionCheck();

	this->last_position_bias = Eigen::VectorXd(GetSkeleton()->getPositions().size()).setZero();
	this->last_velocity_bias = Eigen::VectorXd(GetSkeleton()->getVelocities().size()).setZero();
}

Eigen::VectorXd 
ResponsiveController::
GetEndEffectorStatePosAndVel(const Eigen::VectorXd pos, const Eigen::VectorXd vel) {
	Eigen::VectorXd ret;
	auto& skel = mVirtualCharacter->GetSkeleton();
	BodyNode* root = skel->getRootBodyNode();
	Eigen::Isometry3d cur_root_inv = root->getWorldTransform().inverse();

	int num_ee = mEndEffectors.size();
	Eigen::VectorXd p_save = skel->getPositions();
	Eigen::VectorXd v_save = skel->getVelocities();

	skel->setPositions(pos);
	skel->setVelocities(vel);
	skel->computeForwardKinematics(true, true, false);

	ret.resize((num_ee)*12+15);
//	ret.resize((num_ee)*9+12);

	for(int i=0;i<num_ee;i++)
	{		
		Eigen::Isometry3d transform = cur_root_inv * skel->getBodyNode(mEndEffectors[i])->getWorldTransform();
		//Eigen::Quaterniond q(transform.linear());
		// Eigen::Vector3d rot = QuaternionToDARTPosition(Eigen::Quaterniond(transform.linear()));
		ret.segment<9>(9*i) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
							   transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2), 
							   transform.translation();
//		ret.segment<6>(6*i) << rot, transform.translation();
	}


	for(int i=0;i<num_ee;i++)
	{
	    int idx = skel->getBodyNode(mEndEffectors[i])->getParentJoint()->getIndexInSkeleton(0);
		ret.segment<3>(9*num_ee + 3*i) << vel.segment<3>(idx);
//	    ret.segment<3>(6*num_ee + 3*i) << vel.segment<3>(idx);

	}

	// root diff with target com
	Eigen::Isometry3d transform = cur_root_inv * skel->getRootBodyNode()->getWorldTransform();
	//Eigen::Quaterniond q(transform.linear());

	Eigen::Vector3d rot = QuaternionToDARTPosition(Eigen::Quaterniond(transform.linear()));
	Eigen::Vector3d root_angular_vel_relative = cur_root_inv.linear() * skel->getRootBodyNode()->getAngularVelocity();
	Eigen::Vector3d root_linear_vel_relative = cur_root_inv.linear() * skel->getRootBodyNode()->getCOMLinearVelocity();

	ret.tail<15>() << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
					  transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2),
					  transform.translation(), root_angular_vel_relative, root_linear_vel_relative;
//	ret.tail<12>() << rot, transform.translation(), root_angular_vel_relative, root_linear_vel_relative;

	// restore
	skel->setPositions(p_save);
	skel->setVelocities(v_save);
	//skel->computeForwardKinematics(true, true, false);

	return ret;
}

Eigen::VectorXd
ResponsiveController::
GetState()
{
	// State Component : Joint_angle, Joint_velocity, up_vector_angle, p_next,
	if(mIsTerminal && terminationReason != 8){
		return Eigen::VectorXd::Zero(mNumState);
	}
	SkeletonPtr skel = mVirtualCharacter->GetSkeleton();
	
	double root_height = skel->getRootBodyNode()->getCOM()[1];

	Eigen::VectorXd p_save = skel->getPositions();
	Eigen::VectorXd v_save = skel->getVelocities();
	Eigen::VectorXd p,v;
	// p.resize(p_save.rows()-6);
	// p = p_save.tail(p_save.rows()-6);

	int n_bnodes = mVirtualCharacter->GetSkeleton()->getNumBodyNodes();
	int num_p = (n_bnodes - 1) * 6;
	p.resize(num_p);

	for(int i = 1; i < n_bnodes; i++){
		Eigen::Isometry3d transform = skel->getBodyNode(i)->getRelativeTransform();
		// Eigen::Quaterniond q(transform.linear());
		//	ret.segment<6>(6*i) << rot, transform.translation();
		p.segment<6>(6*(i-1)) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
								 transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2);
	}

	v = v_save;

	BodyNode* root = skel->getRootBodyNode();
	Eigen::Isometry3d cur_root_inv = root->getWorldTransform().inverse();

	Eigen::Vector3d up_vec = root->getTransform().linear()*Eigen::Vector3d::UnitY();
	double up_vec_angle = atan2(std::sqrt(up_vec[0]*up_vec[0]+up_vec[2]*up_vec[2]),up_vec[1]);

	// The angles and velocity of end effector & root info
	Eigen::VectorXd ee;
	ee.resize(mEndEffectors.size()*3);
	for(int i=0;i<mEndEffectors.size();i++)
	{
		Eigen::Isometry3d transform = cur_root_inv * skel->getBodyNode(mEndEffectors[i])->getWorldTransform();
		ee.segment<3>(3*i) << transform.translation();
	}
	double t = mReferenceManager->GetTimeStep(mCurrentFrameOnPhase);

	Motion* p_v_target = mReferenceManager->GetMotion(mCurrentFrame+t);
	Eigen::VectorXd p_now = p_v_target->GetPosition();
	// The rotation and translation of end effector in the future(future 1 frame)
	Eigen::VectorXd p_next = GetEndEffectorStatePosAndVel(p_now, p_v_target->GetVelocity()*t);
	
	delete p_v_target;

	double phase = ((int) mCurrentFrame % mReferenceManager->GetPhaseLength()) / (double) mReferenceManager->GetPhaseLength();
	Eigen::VectorXd state;

	double com_diff = 0;
	
	state.resize(p.rows()+v.rows()+1+1+p_next.rows()+ee.rows()+2);
	state<< p, v, up_vec_angle, root_height, p_next, mAdaptiveStep, ee, mCurrentFrameOnPhase;

	return state;
}

void
ResponsiveController::
SetPDTarget(){
	int num_body_nodes = mInterestedDof / 3;
	int dof = this->mVirtualCharacter->GetSkeleton()->getNumDofs(); 

	for(int i = 0; i < mInterestedDof; i++){
		mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
	}
	int sign = 1;
	if(mActions[mInterestedDof] < 0)
		sign = -1;
	
	mActions[mInterestedDof] = dart::math::clip(mActions[mInterestedDof]*1.2, -2.0, 1.0);
	mActions[mInterestedDof] = exp(mActions[mInterestedDof]);

	this->mPrevFrameOnPhase = this->mCurrentFrameOnPhase;
	this->mCurrentFrame+=1;
	this->mCurrentFrameOnPhase+=1;
	nTotalSteps += 1;
	int n_bnodes = mVirtualCharacter->GetSkeleton()->getNumBodyNodes();

	Motion* p_v_target = mReferenceManager->GetMotion(mCurrentFrame);
	Eigen::VectorXd p_now = p_v_target->GetPosition();
	this->mTargetPositions = p_now ; //p_v_target->GetPosition();

	this->mTargetVelocities = mVirtualCharacter->GetSkeleton()->getPositionDifferences(mTargetPositions, mPrevTargetPositions) / 0.033;
	delete p_v_target;

	p_v_target = mReferenceManager->GetMotion(mCurrentFrame);
	this->mPDTargetPositions = p_v_target->GetPosition();
	this->mPDTargetVelocities = p_v_target->GetVelocity();
	delete p_v_target;


	int count_dof = 0;

	for(int i = 1; i <= num_body_nodes; i++){
		int idx = mVirtualCharacter->GetSkeleton()->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		int dof = mVirtualCharacter->GetSkeleton()->getBodyNode(i)->getParentJoint()->getNumDofs();
		mPDTargetPositions.block(idx, 0, dof, 1) += mActions.block(count_dof, 0, dof, 1);
		count_dof += dof;
	}
}

void
ResponsiveController::
SimStep()
{
	int num_body_nodes = mInterestedDof / 3;

	auto vSkel = mVirtualCharacter->GetSkeleton();

	Eigen::VectorXd torque(vSkel->getNumDofs());

	// implement recovery mode
	if (this->mTargetVelocities.isZero() || this->mPrevTargetPositions.isZero()){
		torque.setZero();
		// auto pos = this->GetSkeleton()->getPositions();
		// pos[0] = NAN;
		// this->GetSkeleton()->setPositions(pos);
		// return;	
	}
	else {
		torque = mVirtualCharacter->GetSPDForces(mPDTargetPositions, mPDTargetVelocities);

		for(int j = 0; j < num_body_nodes; j++) {
			int idx = vSkel->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
			int dof = vSkel->getBodyNode(j)->getParentJoint()->getNumDofs();
			std::string name = mVirtualCharacter->GetSkeleton()->getBodyNode(j)->getName();
			double torquelim = mVirtualCharacter->GetTorqueLimit(name) * 1.5;
			double torque_norm = torque.block(idx, 0, dof, 1).norm();
			
			torque.block(idx, 0, dof, 1) = std::max(-torquelim, std::min(torquelim, torque_norm)) * torque.block(idx, 0, dof, 1).normalized();
		}
	}

	// check contact of humanoid
	bool body_contact = this->HumanoidCollide();

	if (dart::math::isNan(torque) || dart::math::isInf(torque))
		torque.setZero();
	
	// prevent controller exploding by torque
	if (torque.norm() > 10000.0){
		torque /= (torque.norm() / 10000.0);
	}

	mCharacter->GetSkeleton()->setForces(torque);
	mVirtualCharacter->GetSkeleton()->setForces(torque);
	UpdatePerceptionInfo();

	if (body_contact){
		// a contact can occur for a long time, so it could be considered as well in the future
		contact_timestamp.push_back(this->mTimeElapsed);
	}

	mVirtualWorld->step(false);
	mWorld->step(false); 
	

	mTimeElapsed += 1;

	auto vRes = mVirtualWorld->getLastCollisionResult();
	auto res = mWorld->getLastCollisionResult();

	auto collisionSolver = mWorld->getConstraintSolver();
	this->mLastCollision = collisionSolver->getLastCollisionResult();
	this->mLastContacts = getLastContacts();
}

std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, Controller::pair_hash> ResponsiveController::getLastContacts(){
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
bool ResponsiveController::HumanoidCollide (){
	std::vector<dart::dynamics::ShapeNode*> humanoidNodes;
	auto bds = this->mCharacter->GetSkeleton()->getBodyNodes();
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

	auto last_contact = this->mLastContacts;

	for (auto s1 = humanoidNodes.begin(); s1 < humanoidNodes.end(); s1++){
		for (auto s2 = perturbanceNodes.begin(); s2 < perturbanceNodes.end(); s2++){
			auto contact = last_contact.find(std::make_pair((*s1)->getName(), (*s2)->getName()));
			if (contact != last_contact.end() && contact->second.norm() > EPSILON) {
				return true;
			}
		}
	}

	return false;
}

void ResponsiveController::
	UpdatePerceptionInfo()
{
	const int max_reaction_frame = mSimulationHz / 8.0;

	auto skel = mCharacter->GetSkeleton();
	auto vSkel = mVirtualCharacter->GetSkeleton();

	bool update = false;

	if(!mIsFeedbackDelayed) update = true;

	while(!contact_timestamp.empty() && contact_timestamp.front() < this->mTimeElapsed - max_reaction_frame){
		contact_timestamp.erase(contact_timestamp.begin());
		update = true;
		this->recovery_mode = true;
		
		// todo: Properly retarget motion
		mTargetVelocities.setZero();
		mTargetPositions.setZero();
	}

	if (update) {
		Eigen::VectorXd positions = skel->getPositions();
		Eigen::VectorXd velocities = skel->getVelocities();
		vSkel->setPositions(positions);
		vSkel->setVelocities(velocities);
		vSkel->clearConstraintImpulses();
	}
}

void 
ResponsiveController::
ClearRecord() 
{
	Controller::ClearRecord();
	this->mRecordVirtualVelocity.clear();
	this->mRecordVirtualPosition.clear();
	this->mRecordPerturbancePosition.clear();

	// virtual character should be updated before recording in controller reset
	auto vSkel = this->mVirtualCharacter->GetSkeleton();
	vSkel->setPositions(this->GetSkeleton()->getPositions());
	vSkel->setVelocities(this->GetSkeleton()->getVelocities());
}

void
ResponsiveController::
SaveStepInfo() 
{
	Controller::SaveStepInfo();
	mRecordVirtualPosition.push_back(mVirtualCharacter->GetSkeleton()->getPositions());
	mRecordVirtualVelocity.push_back(mVirtualCharacter->GetSkeleton()->getVelocities());
	vector<Eigen::VectorXd> pPositions;
	for(auto p : perturbance){
		pPositions.push_back(p->getPositions());
	}
	mRecordPerturbancePosition.push_back(pPositions);
}

void
ResponsiveController::
Reset(bool RSI)
{
	this->recovery_mode = false;
	this->mVirtualWorld->reset();
	auto vSkel = this->mVirtualCharacter->GetSkeleton();
	vSkel->clearConstraintImpulses();
	vSkel->clearInternalForces();
	vSkel->clearExternalForces();
	d_expected_positions.clear();
	d_expected_velocities.clear();
	contact_timestamp.clear();
	last_position_bias.setZero();
	last_velocity_bias.setZero();
	d_position_bias.setZero();
	this->mLastCollision.clear();

	Controller::Reset(RSI);
	// clearrecord has functions needed in reset
}

}