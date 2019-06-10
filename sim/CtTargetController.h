#pragma once

#include "sim/CtController.h"

/*
 * This controler is a "target" controller
 * By this we mean that part of the state includes a footstep plan
 * that may be used as part of the reward.
 *
 *
 */
class cCtTargetController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtTargetController();
	virtual ~cCtTargetController();

	virtual void Init(cSimCharacter* character);
	virtual void SetTargetPos(const tVector& pos);

protected:
	tVector mTargetPos;

	virtual void InitTargetPos();
	virtual int GetPoliStateSize() const;
	virtual int GetTargetPosStateOffset() const;
	virtual int GetTargetPosStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildTargetPosState(Eigen::VectorXd& out_state) const;
};
