#pragma once
#include "sim/BipedStepController3D.h"

class cBipedSymStepController3D : public virtual cBipedStepController3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	enum eSymTaskParam
	{
		eSymTaskParamRootHeading,
		eSymTaskParamStepX0,
		eSymTaskParamStepY0,
		eSymTaskParamStepZ0,
		eSymTaskParamStepX1,
		eSymTaskParamStepY1,
		eSymTaskParamStepZ1,
		eSymTaskParamMax
	};
	typedef Eigen::Matrix<double, eSymTaskParamMax, 1> tSymTaskParams;

	cBipedSymStepController3D();
	virtual ~cBipedSymStepController3D();

	virtual const std::vector<int>& GetFlipJointOrder() const;

protected:

	std::vector<int> mFlipJointOrder;

	virtual void InitEndEffectors();

	virtual void BuildPhaseState(Eigen::VectorXd& out_state) const;
	virtual void BuildTaskState(Eigen::VectorXd& out_state) const;
	virtual int GetTaskStateSize() const;

	virtual int RetargetJointID(int joint_id) const;
	virtual bool FlipStance() const;

#if defined(ENABLE_PHASE_STATE_BINS)
	virtual int GetNumPhaseBins() const;
#endif // ENABLE_PHASE_STATE_BINS
};