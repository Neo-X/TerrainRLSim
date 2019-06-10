#include "sim/BipedSymStepController3D.h"
#include "sim/SimCharacter.h"

const int gNumPhaseBins = 2;

cBipedSymStepController3D::cBipedSymStepController3D() : cBipedStepController3D()
{
}

cBipedSymStepController3D::~cBipedSymStepController3D()
{
}

const std::vector<int>& cBipedSymStepController3D::GetFlipJointOrder() const
{
	return mFlipJointOrder;
}

void cBipedSymStepController3D::InitEndEffectors()
{
	cBipedStepController3D::InitEndEffectors();

	int num_joints = mChar->GetNumJoints();
	mFlipJointOrder.resize(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		mFlipJointOrder[j] = j;
	}

	assert(mEndEffectors.size() == 2);
	int right_j = mEndEffectors[0];
	int left_j = mEndEffectors[1];
	while (left_j != right_j && left_j != gInvalidIdx && right_j != gInvalidIdx)
	{
		mFlipJointOrder[right_j] = left_j;
		mFlipJointOrder[left_j] = right_j;
		right_j = mChar->GetParentJoint(right_j);
		left_j = mChar->GetParentJoint(left_j);
	}
}

void cBipedSymStepController3D::BuildPhaseState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetPhaseStateSize());

	double phase = std::fmod(2 * GetPhase(), 1.0);
#if defined(ENABLE_COS_PHASE)
	int dphase_offset = 2;
	double theta = 2 * M_PI * phase;
	out_state[0] = std::cos(theta);
	out_state[1] = std::sin(theta);
#else
	int dphase_offset = 1;
	out_state[0] = phase;
#endif

#if defined(ENABLE_PHASE_STATE_BINS)
	int bin = static_cast<int>(phase * GetNumPhaseBins());
	out_state[bin + dphase_offset] = 1;
#endif // ENABLE_PHASE_STATE_BINS
}

void cBipedSymStepController3D::BuildTaskState(Eigen::VectorXd& out_state) const
{
	out_state = tSymTaskParams::Zero();
	double tar_heading = mStepPlan.mRootHeading;
	tVector heading_dir = tVector(std::cos(tar_heading), 0, -std::sin(tar_heading), 0);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix heading_trans = mChar->BuildOriginTrans();

	if (FlipStance())
	{
		heading_trans.row(2) *= -1; // reflect z
	}

	heading_dir = heading_trans * heading_dir;
	tar_heading = std::atan2(-heading_dir[2], heading_dir[0]);

	bool right_stance = mStepPlan.mStance == eStanceRight;
	int swing_id = (right_stance) ? mEndEffectors[eStanceLeft] : mEndEffectors[eStanceRight];
	tVector swing_pos = mChar->CalcJointPos(swing_id);
	tVector pos0 = mStepPlan.mStepPos0;
	tVector delta0 = pos0 - swing_pos;
	delta0 = heading_trans * delta0;
	delta0[1] = pos0[1] - ground_h;

	int stance_id = (right_stance) ? mEndEffectors[eStanceRight] : mEndEffectors[eStanceLeft];
	tVector stance_pos = mChar->CalcJointPos(stance_id);
	tVector pos1 = mStepPlan.mStepPos1;
	tVector delta1 = pos1 - stance_pos;
	delta1 = heading_trans * delta1;
	delta1[1] = pos1[1] - ground_h;

	out_state[eSymTaskParamRootHeading] = tar_heading;
	out_state[eSymTaskParamStepX0] = delta0[0];
	out_state[eSymTaskParamStepY0] = delta0[1];
	out_state[eSymTaskParamStepZ0] = delta0[2];
	out_state[eSymTaskParamStepX1] = delta1[0];
	out_state[eSymTaskParamStepY1] = delta1[1];
	out_state[eSymTaskParamStepZ1] = delta1[2];
}

int cBipedSymStepController3D::GetTaskStateSize() const
{
	return eSymTaskParamMax;
}

int cBipedSymStepController3D::RetargetJointID(int joint_id) const
{
	int new_joint_id = joint_id;
	if (FlipStance())
	{
		new_joint_id = mFlipJointOrder[joint_id];
	}
	return new_joint_id;
}

bool cBipedSymStepController3D::FlipStance() const
{
	eStance stance = GetStance();
	bool flip_stance = stance == eStanceLeft;
	return flip_stance;
}

#if defined(ENABLE_PHASE_STATE_BINS)
int cBipedSymStepController3D::GetNumPhaseBins() const
{
	return gNumPhaseBins;
}
#endif // ENABLE_PHASE_STATE_BINS