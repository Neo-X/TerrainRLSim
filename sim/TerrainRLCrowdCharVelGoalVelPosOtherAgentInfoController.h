#pragma once


#include "TerrainRLCrowdCharVelGoalVelPosController.h"

class TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController : public cTerrainRLCrowdCharVelGoalVelPosController
{
public:

	TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController();
	virtual ~TerrainRLCrowdCharVelGoalVelPosOtherAgentInfoController();

	void BuildTargetPosState(Eigen::VectorXd& out_state) const;

protected:

	virtual int GetTargetPosStateSize() const;

};
