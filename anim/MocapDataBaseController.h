#pragma once

#include "anim/MocapStepController.h"
#include "anim/Motion.h"
#include "sim/BipedStepController3D.h"

class cMocapDataBaseController : public cMocapStepController
{
public:
	
	cMocapDataBaseController();
	virtual ~cMocapDataBaseController();

	virtual void Reset();
	virtual void Update(double time_step);

	virtual size_t getMotionID() const;
	virtual void setMotionID(size_t id);

protected:

	virtual int SelectNewMotion() const;
	virtual void MirrorPoseStance(Eigen::VectorXd& out_pose) const;

	virtual void UpdateNewStep();
	virtual double CalcMotionPhase(double time) const;

	virtual void ResetParams();
	bool randomMotions;

	
};
