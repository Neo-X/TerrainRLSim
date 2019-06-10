#pragma once

#include "WaypointVelController.h"

class cMultiCharWaypointVelController : public cWaypointVelController
{
public:

	cMultiCharWaypointVelController();
	virtual ~cMultiCharWaypointVelController();

	void BuildTargetPosState(Eigen::VectorXd& out_state) const;
	void ApplyAction(const tAction& action);
	// void HandleNewActionUpdate();
	// bool NewActionUpdate() const;

protected:


};
