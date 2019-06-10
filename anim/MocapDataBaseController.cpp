#include "anim/MocapDataBaseController.h"
#include "anim/KinCharacter.h"
#include "util/FileUtil.h"
// #include "util/MathUtil"


cMocapDataBaseController::cMocapDataBaseController() :
	cMocapStepController()
{

}

cMocapDataBaseController::~cMocapDataBaseController()
{
}

void cMocapDataBaseController::Reset()
{
	cKinController::Reset();
	ResetParams();

	mCurrMotionID = SelectNewMotion();
	UpdateNewStep();
}


void cMocapDataBaseController::Update(double time_step)
{
	cKinController::Update(time_step);
	
}

int cMocapDataBaseController::SelectNewMotion() const
{
	return cMathUtil::RandInt(0,GetNumMotions());
}

void cMocapDataBaseController::MirrorPoseStance(Eigen::VectorXd& out_pose) const
{
	/// Don't mirror things
}

size_t cMocapDataBaseController::getMotionID() const
{
	return mCurrMotionID;
}

void cMocapDataBaseController::UpdateNewStep()
{

	const auto& joint_mat = mChar->GetJointMat();
	Eigen::VectorXd new_pose;
	CalcMotionPose(mTime, mCurrMotionID, new_pose);

	tQuaternion root_rot = mChar->CalcHeadingRot();
	tVector root_pos = mChar->GetRootPos();
	tQuaternion offset_root_rot = cKinTree::CalcHeadingRot(joint_mat, new_pose);
	tVector offset_root_pos = cKinTree::GetRootPos(joint_mat, new_pose);
	tQuaternion char_origin_rot = mChar->GetOriginRot();
	tVector char_origin_trans = mChar->GetOriginPos();

	mOriginRot = root_rot;
	mOriginRot = mOriginRot * offset_root_rot.inverse();
	mOriginRot = cMathUtil::QuatDiff(char_origin_rot, mOriginRot);

	mOriginTrans = root_pos - cMathUtil::QuatRotVec(root_rot * offset_root_rot.inverse(), offset_root_pos);
	mOriginTrans[1] = 0;
	mOriginTrans -= char_origin_trans;
	mOriginTrans = cMathUtil::QuatRotVec(char_origin_rot.inverse(), mOriginTrans);
}

double cMocapDataBaseController::CalcMotionPhase(double time) const
{
	double motionDuration = mMotions[mCurrMotionID].GetDuration();
	// std::cout << "motionDuration: " << motionDuration << std::endl;
	double phase = time / motionDuration;
	// double phase = time / mCyclePeriod;
	phase -= static_cast<int>(phase);
	phase = (phase < 0) ? (1 + phase) : phase;
	return phase;
}

