#ifndef __REACTIVE_CONTROLLER_H__
#define __REACTIVE_CONTROLLER_H__
#include "Controller.h"
namespace DPhy
{

class ResponsiveController: public Controller
{
public:
	ResponsiveController(ReferenceManager* ref, const std::string character_path, bool record=false, int id=0);

	void SimStep() override;

	void setFeedbackDelayed(bool val){ this->mIsFeedbackDelayed = val; }
	bool getFeedbackDelayed(){ return this->mIsFeedbackDelayed; }

	bool HumanoidCollide ();
	void UpdatePerceptionInfo();

	Eigen::VectorXd GetVirtualPositions(int idx) { return this->mRecordVirtualPosition[idx]; }
	Eigen::VectorXd GetVirtualVelocities(int idx) { return this->mRecordVirtualVelocity[idx]; }
	std::vector<Eigen::VectorXd> GetPerturbancePositions(int idx) { return this->mRecordPerturbancePosition[idx]; }
	Eigen::VectorXd GetEndEffectorStatePosAndVel(const Eigen::VectorXd pos, const Eigen::VectorXd vel) override;

	void SetPDTarget() override;
	Eigen::VectorXd GetState() override;
	void UpdatePerturbance(std::vector<dart::dynamics::SkeletonPtr> vec){ perturbance = vec; }

	void ClearRecord() override;
	void SaveStepInfo() override;

	void Reset(bool RSI);
	
	std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, pair_hash> getLastContacts();

private:
	bool recovery_mode = false;
	dart::collision::CollisionResult mLastCollision;
	std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, Controller::pair_hash> mLastContacts;
	// Difference between the actual state and state without contact
	// Given that controller is taking actions based on its perceived state
	std::vector<Eigen::VectorXd> d_expected_positions;
	std::vector<Eigen::VectorXd> d_expected_velocities;
	std::vector<int> contact_timestamp;

	Eigen::VectorXd last_position_bias;
	Eigen::VectorXd last_velocity_bias;
	Eigen::VectorXd d_position_bias;

	// Weighted some of the delta expected states is added to the actual state.
	Eigen::VectorXd perceived_positions;
	Eigen::VectorXd perceived_velocities;

	// virtual world to simulate perceived world
	dart::simulation::WorldPtr mVirtualWorld;
	Character *mVirtualCharacter;
	std::vector<dart::dynamics::SkeletonPtr> perturbance;

	std::vector<Eigen::VectorXd> mRecordVirtualPosition;
	std::vector<Eigen::VectorXd> mRecordVirtualVelocity;
	std::vector<std::vector<Eigen::VectorXd>> mRecordPerturbancePosition;

	bool mIsFeedbackDelayed;
};

}

#endif