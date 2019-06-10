#pragma once

#include "WaypointController.h"

class cMultiCharWaypointController : public cWaypointController
{
public:

	cMultiCharWaypointController();
	virtual ~cMultiCharWaypointController();

	void BuildTargetPosState(Eigen::VectorXd& out_state) const;
	void ApplyAction(const tAction& action);
	// void HandleNewActionUpdate();
	// bool NewActionUpdate() const;

protected:


};
