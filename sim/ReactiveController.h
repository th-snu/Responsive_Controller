#ifndef __REACTIVE_CONTROLLER_H__
#define __REACTIVE_CONTROLLER_H__
#include "Controller.h"
namespace DPhy
{

class ReactiveController: public Controller
{
public:
	ReactiveController(ReferenceManager* ref, const std::string character_path, bool record=false, int id=0);

	void SimStep() override;

	void setFeedbackDelayed(bool val){ this->mIsFeedbackDelayed = val; }
	bool getFeedbackDelayed(){ return this->mIsFeedbackDelayed; }

	bool HumanoidCollide ();
	void UpdatePerceptionInfo();

	Eigen::VectorXd GetVirtualPositions(int idx) { return this->mRecordVirtualPosition[idx]; }
	Eigen::VectorXd GetVirtualVelocities(int idx) { return this->mRecordVirtualVelocity[idx]; }
	Eigen::VectorXd GetEndEffectorStatePosAndVel(const Eigen::VectorXd pos, const Eigen::VectorXd vel) override;

	void SetPDTarget() override;
	Eigen::VectorXd GetState() override;
	void UpdatePerturbance(std::vector<dart::dynamics::SkeletonPtr> vec){ perturbance = vec; }

	void ClearRecord() override;
	void SaveStepInfo() override;
	
	std::unordered_map<std::pair<std::string, std::string>, Eigen::Vector3d, pair_hash> getLastContacts();

private:
	// Difference between the actual state and state without contact
	// Given that controller is taking actions based on its perceived state
	std::vector<Eigen::VectorXd> d_expected_positions;
	std::vector<Eigen::VectorXd> d_expected_velocities;
	std::vector<int> contact_timestamp;

	// Weighted some of the delta expected states is added to the actual state.
	Eigen::VectorXd perceived_positions;
	Eigen::VectorXd perceived_velocities;

	// virtual world to simulate perceived world
	dart::simulation::WorldPtr mVirtualWorld;
	Character *mVirtualCharacter;
	std::vector<dart::dynamics::SkeletonPtr> perturbance;

	std::vector<Eigen::VectorXd> mRecordVirtualPosition;
	std::vector<Eigen::VectorXd> mRecordVirtualVelocity;

	bool mIsFeedbackDelayed;
};

}

#endif