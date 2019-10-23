#pragma once


#include "TerrainRLCrowdCharVelController.h"

class cTerrainRLCrowdCharVelGoalVelController : public cTerrainRLCrowdCharVelController
{
public:

	cTerrainRLCrowdCharVelGoalVelController();
	virtual ~cTerrainRLCrowdCharVelGoalVelController();

	void BuildTargetPosState(Eigen::VectorXd& out_state) const;

protected:

	virtual int GetTargetPosStateSize() const;

};
