#include "sim/WaypointControllerGeneral.h"
#include "sim/SimCharacter.h"
#include "sim/BipedSymStepController3D.h"
#include "util/FileUtil.h"

//#define ENABLE_BILINEAR_PHASE
//#define HACK_OUTPUT_CONV_WEIGHTS

const double gMaxWaypointDist = 2;
const double gMaxStepDist0 = 2;
const double gMaxStepDist1 = 4;
const double gMaxHeading = M_PI;
const int gGroundSampleRes = 32;
const int gHeadingStateDim = 2;
const int gNumPhaseBins = 2;

cWaypointControllerGeneral::cWaypointControllerGeneral() : cCtTargetController()
{
	mViewDist = 10;
	mViewDistMin = -1;
	mCurrTimeStep = 0;
	mNewActionUpdate = false;
	mInitStepLen = 0;
	mSymmetricStep = false;
	mFlipJointOrder = nullptr;
}

cWaypointControllerGeneral::~cWaypointControllerGeneral()
{
}

void cWaypointControllerGeneral::Init(cSimCharacter* character)
{
	cCtTargetController::Init(character);
}

void cWaypointControllerGeneral::Reset()
{
	cCtTargetController::Reset();
}

void cWaypointControllerGeneral::Clear()
{
	cCtTargetController::Clear();
}

void cWaypointControllerGeneral::Update(double time_step)
{
	mCurrTimeStep = time_step;
	cCtTargetController::Update(time_step);
}

void cWaypointControllerGeneral::SetGround(std::shared_ptr<cGround> ground)
{
	cCtTargetController::SetGround(ground);
}

void cWaypointControllerGeneral::SetInitStepLen(double step_len)
{
	mInitStepLen = step_len;
}

int cWaypointControllerGeneral::GetPoliStateSize() const
{
	int size = cCtTargetController::GetPoliStateSize();
#if defined(ENABLE_BILINEAR_PHASE)
	size += GetPhaseStateSize();
#endif
	return size;
}

void cWaypointControllerGeneral::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtTargetController::BuildNNInputOffsetScaleTypes(out_types);

#if defined(ENABLE_BILINEAR_PHASE)
	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	for (int i = 0; i < phase_size; ++i)
	{
		out_types[phase_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
#endif
}

void cWaypointControllerGeneral::GetViewBound(tVector& out_min, tVector& out_max) const
{
	cCtTargetController::GetViewBound(out_min, out_max);

}

void cWaypointControllerGeneral::EnableExp(bool enable)
{
	cCtTargetController::EnableExp(enable);
}

void cWaypointControllerGeneral::SetTime(double time)
{
	ApplyAction(mCurrAction);
}

void cWaypointControllerGeneral::ResetParams()
{
	cCtTargetController::ResetParams();
	mNewActionUpdate = false;
}

void cWaypointControllerGeneral::PostProcessAction(tAction& out_action) const
{
	//out_action.mParams = out_action.mParams.cwiseMax(mActionBoundMin).cwiseMin(mActionBoundMax);
}

void cWaypointControllerGeneral::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(action_size);
	out_scale = Eigen::VectorXd::Ones(action_size);

	out_offset[eActionParamStepX0] = -mInitStepLen;
	out_offset[eActionParamStepX1] = -2 * mInitStepLen;

	out_scale[eActionParamStepX0] = 2 / gMaxStepDist0;
	out_scale[eActionParamStepZ0] = 2 / gMaxStepDist0;
	out_scale[eActionParamStepX1] = 2 / gMaxStepDist1;
	out_scale[eActionParamStepZ1] = 2 / gMaxStepDist1;
	out_scale[eActionParamRootHeading] = 1 / gMaxHeading;
	//out_scale[eActionParamRootHeading] *= 0.5;
}

void cWaypointControllerGeneral::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	out_tau = Eigen::VectorXd::Zero(mChar->GetNumDof());
}

void cWaypointControllerGeneral::UpdateNewActionUpdate(double time_step)
{
	mNewActionUpdate = !std::isfinite(mUpdateCounter);
}

bool cWaypointControllerGeneral::NewActionUpdate() const
{
	return true;
}

void cWaypointControllerGeneral::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	UpdateNewActionUpdate(time_step);
	mUpdateCounter = 0;

	/// Hack Hack this might break the old HLCs
	if (NewActionUpdate() && false)
	{
		UpdateAction();
		mFirstCycle = false;
	}

	UpdateBuildTau(time_step, out_tau);
}

void cWaypointControllerGeneral::DecideAction(tAction& out_action)
{
	cCtTargetController::DecideAction(out_action);
	/*
	// hack
	FILE* f = cFileUtil::OpenFile("output/terrain_map.txt", "w");
	for (int i = 0; i < mGroundSamples.size(); ++i)
	{
		double val = mGroundSamples[i];
		fprintf(f, "%.5f\t", val);
	}
	cFileUtil::CloseFile(f);
	*/

#if defined(HACK_OUTPUT_CONV_WEIGHTS)
	const auto& params = mNet->GetParams();
	//const auto& params = mCriticNet->GetParams();
	const auto& params0 = params[0];
	const auto* data = params0->cpu_data();

	FILE* f = cFileUtil::OpenFile("output/conv_filters.txt", "w");
	for (int i = 0; i < params0->count(); ++i)
	{
		double val = data[i];
		fprintf(f, "%.5f\n", val);
	}
	cFileUtil::CloseFile(f);
#endif
}

void cWaypointControllerGeneral::ApplyAction(const tAction& action)
{
	cCtTargetController::ApplyAction(action);
}

int cWaypointControllerGeneral::GetGroundSampleRes() const
{
	return gGroundSampleRes;
}

void cWaypointControllerGeneral::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtTargetController::BuildPoliState(out_state);

#if defined(ENABLE_BILINEAR_PHASE)
	Eigen::VectorXd phase_state;
	BuildPhaseState(phase_state);

	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	out_state.segment(phase_offset, phase_size) = phase_state;
#endif
}

int cWaypointControllerGeneral::GetPhaseStateOffset() const
{
	return cCtTargetController::GetPoliStateSize();
}

int cWaypointControllerGeneral::GetPhaseStateSize() const
{
	return gNumPhaseBins;
}

void cWaypointControllerGeneral::BuildPhaseState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetPhaseStateSize());
	out_state[0] = 0;
	out_state[1] = 0;
}

bool cWaypointControllerGeneral::EnableSymStep() const
{
	return mSymmetricStep;
}

int cWaypointControllerGeneral::RetargetJointID(int joint_id) const
{
	int new_joint_id = joint_id;
	if (FlipStance())
	{
		new_joint_id = (*mFlipJointOrder)[joint_id];
	}
	return new_joint_id;
}

bool cWaypointControllerGeneral::FlipStance() const
{
	return 0;
}
