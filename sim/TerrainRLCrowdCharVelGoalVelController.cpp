#include "TerrainRLCrowdCharVelGoalVelController.h"


cTerrainRLCrowdCharVelGoalVelController::cTerrainRLCrowdCharVelGoalVelController() : cTerrainRLCrowdCharVelController()
{
}

cTerrainRLCrowdCharVelGoalVelController::~cTerrainRLCrowdCharVelGoalVelController()
{
}

void cTerrainRLCrowdCharVelGoalVelController::BuildTargetPosState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());
	tVector target_pos = mChar->GetCurrentGroundTarget();
	tVector target_vel = mChar->GetCurrentGroundTargetVel();
	target_pos[3] = 1;
	tMatrix origin_trans = mChar->BuildOriginTrans();
	target_pos = origin_trans * target_pos;
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
	out_state[2] = target_vel[0]; // x vel
	out_state[3] = target_vel[2]; // z vel
}

int cTerrainRLCrowdCharVelGoalVelController::GetTargetPosStateSize() const
{

	return 4;
}

/*
bool cMultiCharWaypointController::NewActionUpdate() const
{
	return false;
}
*/
