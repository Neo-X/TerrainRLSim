#include "TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController.h"


TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController::TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController() : cTerrainRLCrowdCharVelGoalVelPosController()
{
}

TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController::~TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController()
{
}

void TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController::BuildTargetPosState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());
	tVector target_pos_ = mChar->GetCurrentGroundTarget();
	tVector target_vel = mChar->GetCurrentGroundTargetVel();
	target_pos_[3] = 1;
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tVector target_pos = origin_trans * target_pos_;
	target_pos[1] = 0;
	target_pos[3] = 0;

	if (FlipStance())
	{
		target_pos[2] = -target_pos[2];
	}

	double tar_theta = std::atan2(-target_pos[2], target_pos[0]);
	double tar_dist = target_pos.norm();

	out_state[0] = tar_theta;
	out_state[1] = tar_dist;
	out_state[2] = target_pos_[0]; // x pos
	out_state[3] = target_pos_[2]; // z pos
	out_state[4] = target_vel[0]; // x vel
	out_state[5] = target_vel[2]; // z vel

	/// Add other agent pos a velocity
	for (size_t a=0; a < mGround->GetChars().size(); a++)
	{
//		std::cout << "mGround->GetChars()[a]->CalcCOM(): " << mGround->GetChars()[a]->CalcCOM() << std::endl;
		tVector target_pos__ = mGround->GetChars()[a]->CalcCOM() - mChar->CalcCOM();
		tVector target_vel_ = mGround->GetChars()[a]->CalcCOMVel() - mChar->CalcCOMVel();
		out_state[6+(a*4)+0] = target_pos__[0];
		out_state[6+(a*4)+1] = target_pos__[2];
		out_state[6+(a*4)+2] = target_vel_[0];
		out_state[6+(a*4)+3] = target_vel_[2];
	}
//	std::cout << "agent goal state: " << out_state << std::endl;
}

int TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController::GetTargetPosStateSize() const
{

	return 6 + mGround->GetChars().size() * 4;
}

/*
bool cMultiCharWaypointController::NewActionUpdate() const
{
	return false;
}
*/
