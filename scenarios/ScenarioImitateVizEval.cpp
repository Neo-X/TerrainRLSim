#include "ScenarioImitateVizEval.h"
#include "scenarios/ScenarioExpImitateViz.h"
#include "sim/CtTrackController.h"
#include "sim/CtPhaseController.h"
#include "sim/WaypointController.h"
#include "sim/SimCharSoftFall.h"
#include "util/FileUtil.h"

cScenarioImitateVizEval::cScenarioImitateVizEval() :
	cScenarioExpImitateViz(),
	cScenarioPoliEval()
{
	/// Enable this for python training
	mEnableRandStateReset = true;
	mRecordPoseError = true;
	mPoseErr = 0;
	mPoseErrCount = 0;

	SetResetPhase(0, 1);
	SetResetPhaseSamples(20);
	// EnableTargetPos(true);
}

cScenarioImitateVizEval::~cScenarioImitateVizEval()
{
}

void cScenarioImitateVizEval::GetPoseErrResult(double& out_err, int& out_count) const
{
	out_err = mPoseErr;
	out_count = mPoseErrCount;
}

void cScenarioImitateVizEval::SetResetPhase(double phase_min, double phase_max)
{
	mResetPhaseMin = phase_min;
	mResetPhaseMax = phase_max;
}

void cScenarioImitateVizEval::SetResetPhaseSamples(int num_samples)
{
	mResetPhaseSamples = num_samples;
}

std::string cScenarioImitateVizEval::GetName() const
{
	return "Imitate Evaluation";
}

void cScenarioImitateVizEval::ResetRecord()
{
	cScenarioPoliEval::ResetRecord();
	mPoseErr = 0;
	mPoseErrCount = 0;
}

void cScenarioImitateVizEval::UpdatePoseErr()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& kin_pose = mKinChar->GetPose();
	
	Eigen::VectorXd pose_diff = kin_pose - pose;
	int num_joints = mChar->GetNumJoints();
	int root_id = mChar->GetRootID();
	const auto& joint_desc = mChar->GetJointMat();
	double pose_err = 0;

	tVector root_diff = cKinTree::GetRootPos(joint_desc, pose_diff);
	root_diff[0] = 0;
	cKinTree::SetRootPos(joint_desc, root_diff, pose_diff);

	for (int j = 0; j < num_joints; ++j)
	{
		int offset = mChar->GetParamOffset(j);
		int size = mChar->GetParamSize(j);
		double w = mJointWeights[j];

		double curr_pose_diff = pose_diff.segment(offset, size).squaredNorm();
		pose_err += w * curr_pose_diff;
	}

	mPoseErr = cMathUtil::AddAverage(mPoseErr, mPoseErrCount, pose_err, 1);
	++mPoseErrCount;

#if defined (ENABLE_DEBUG_PRINT)
	if (mPoseErrCount % 1000 == 0)
	{
		printf("Avg Pose Err: %.5f (%i)\n", mPoseErr, mPoseErrCount);
	}
#endif
}

void cScenarioImitateVizEval::RecordAction(const std::string& out_file)
{
	auto ctrl = std::static_pointer_cast<cTerrainRLCharController>(mChar->GetController());

	std::string data_str = "";
	double phase = mKinChar->GetPhase();
	data_str += std::to_string(phase);

	Eigen::VectorXd action;
	ctrl->RecordPoliAction(action);

	int data_size = static_cast<int>(action.size());
	for (int i = 0; i < data_size; ++i)
	{
		data_str += ",\t";
		data_str += std::to_string(action[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

void cScenarioImitateVizEval::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioPoliEval::ParseMiscArgs(parser);
	parser->ParseBool("record_pose_err", mRecordPoseError);
}

void cScenarioImitateVizEval::InitMisc()
{
	cScenarioPoliEval::InitMisc();
	mPoseErr = 0;
	mPoseErrCount = 0;
}

void cScenarioImitateVizEval::ResetMisc()
{
	cScenarioPoliEval::ResetMisc();
	mPosStart = mChar->GetRootPos();
}

void cScenarioImitateVizEval::ClearMisc()
{
	cScenarioPoliEval::ClearMisc();
	mPoseErr = 0;
	mPoseErrCount = 0;
}

void cScenarioImitateVizEval::UpdateMisc(double time_elapsed)
{
	cScenarioPoliEval::UpdateMisc(time_elapsed);
	if (mRecordPoseError)
	{
		UpdatePoseErr();
	}
}

double cScenarioImitateVizEval::CalcRandKinResetTime()
{
	double dur = mCtrlParams.mCycleDur;
	// double phase = static_cast<double>(mEpisodeCount % mResetPhaseSamples) / mResetPhaseSamples;
	// phase = phase * (mResetPhaseMax - mResetPhaseMin) + mResetPhaseMin;
	// double rand_time = phase * dur;
	double rand_time = mRand.RandDouble(0, dur);
	// std::cout << "reset phase time: " << rand_time << std::endl;
	return rand_time;
}
