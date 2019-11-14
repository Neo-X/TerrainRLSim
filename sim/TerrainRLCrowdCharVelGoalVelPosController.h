#pragma once


#include "TerrainRLCrowdCharVelGoalVelController.h"

class cTerrainRLCrowdCharVelGoalVelPosController : public cTerrainRLCrowdCharVelGoalVelController
{
public:

	cTerrainRLCrowdCharVelGoalVelPosController();
	virtual ~cTerrainRLCrowdCharVelGoalVelPosController();

	void BuildTargetPosState(Eigen::VectorXd& out_state) const;

protected:

	virtual int GetTargetPosStateSize() const;

};
