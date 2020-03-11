#include "scenarios/ScenarioExpHikeGeneral.h"
#include "sim/WaypointControllerGeneral.h"
#include "learning/CaclaTrainer.h"

const int gNumLLCWarmupCycles = 0;

double cScenarioExpHikeGeneral::CalcReward() const
{
	const double target_speed = mTargetSpeed;

	const tVector& prev_com = mPrevCOM;
	double time_elapsed = mTime - mPrevTime;
	// std::cout << "mTime: " << mTime << " mPrevTime: " << mPrevTime << std::endl;
	double vel_w = 0.8;
	double step_w = 0 ;
	double heading_w = 0;
	double LLC_reward_w = 0 ;
	double fixed_w = 0.2;

	const double total_w = vel_w + step_w + heading_w + LLC_reward_w + fixed_w;
	vel_w /= total_w;
	step_w /= total_w;
	heading_w /= total_w;
	LLC_reward_w /= total_w;
	fixed_w /= total_w;

	const double vel_scale = 1.5;
	const double step_scale = 5;

	double reward = 0;

	if (time_elapsed > 0)
	{
		bool fallen = HasFallen();
		if (!fallen)
		{
			tVector curr_com = mChar->CalcCOM();
			tVector com_delta = curr_com - prev_com;
			tVector target_delta = mTargetPos - prev_com;
			com_delta[1] = 0;
			target_delta[1] = 0;
			tVector target_dir = target_delta.normalized();

			double avg_vel = target_dir.dot(com_delta);
			avg_vel /= time_elapsed;
			double vel_err = std::min(0.0, avg_vel - target_speed);
			double target_dist_threshold = GetTargetResetDist();
			if (target_delta.squaredNorm() < target_dist_threshold * target_dist_threshold)
			{
				vel_err = 0;
			}
			vel_err *= vel_err;

			eStance stance = GetStance();
			int stance_foot_id = GetStanceFootJoint(stance);
			tVector stance_foot_pos = mChar->CalcJointPos(stance_foot_id);
			const tVector& step_pos = mStepPlan.mStepPos0;
			tVector step_delta = step_pos - stance_foot_pos;
			double step_err = step_delta.squaredNorm();

			double heading = mChar->CalcHeading();
			double tar_heading = mStepPlan.mRootHeading;
			double heading_err = std::abs(tar_heading - heading);
			heading_err = std::min(2 * M_PI - heading_err, heading_err);


			double vel_reward = std::exp(-vel_scale * vel_err);
			//vel_reward = (avg_vel > 0) ? vel_reward : 0;
			double step_reward = exp(-step_scale * step_err);
			double heading_reward = 0.5 * (std::cos(heading_err) + 1);
			heading_reward = std::pow(heading_reward, 4);

			reward = vel_w * vel_reward + step_w * step_reward + heading_w * heading_reward
					+ fixed_w;
		}
	}
	return reward;
}

cScenarioExpHikeGeneral::cScenarioExpHikeGeneral()
{
	mTargetSpeed = 1;
	mTargetResetDist = 1;

	mCharParams.mEnableSoftContact = true;
	EnableTargetPos(true);
	EnableRandTargetPos(true);
}

cScenarioExpHikeGeneral::~cScenarioExpHikeGeneral()
{
}

void cScenarioExpHikeGeneral::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpImitateStep::ParseArgs(parser);

	parser->ParseDouble("target_speed", mTargetSpeed);

}

void cScenarioExpHikeGeneral::Init()
{
	cScenarioExpImitateStep::Init();

	if (EnabledRandStateReset())
	{
		ResetTargetPos();
	}

	InitMisc();
}

void cScenarioExpHikeGeneral::Reset()
{
	cScenarioExpImitateStep::Reset();

	if (EnabledRandStateReset())
	{
		ResetTargetPos();
	}

	ResetMisc();
}

void cScenarioExpHikeGeneral::Clear()
{
	cScenarioExpImitateStep::Clear();
}

void cScenarioExpHikeGeneral::SetBufferSize(int size)
{
	cScenarioExpImitateStep::SetBufferSize(size);
}

void cScenarioExpHikeGeneral::EnableExplore(bool enable)
{
	cScenarioExpImitateStep::EnableExplore(enable);
}

std::string cScenarioExpHikeGeneral::GetName() const
{
	return "Hike Exploration";
}

void cScenarioExpHikeGeneral::ResetParams()
{
	cScenarioExpImitateStep::ResetParams();
}

void cScenarioExpHikeGeneral::ResetKinChar()
{
	mKinChar->Reset();
	if (EnabledRandStateReset())
	{
		const double phase_offset = 0.1;

		double dur = mCtrlParams.mCycleDur;
		double rand_phase = (mRand.FlipCoin()) ? 0 : 0.5;
		rand_phase += phase_offset;
		double rand_time = rand_phase * dur;

		mKinChar->SetTime(rand_time);
		mKinChar->Pose(rand_time);
	}
}

void cScenarioExpHikeGeneral::PreSubstepUpdate(double time_step)
{
	cScenarioExpImitateStep::PreSubstepUpdate(time_step);
}

void cScenarioExpHikeGeneral::PostSubstepUpdate(double time_step)
{
	cScenarioExpImitateStep::PostSubstepUpdate(time_step);
}

void cScenarioExpHikeGeneral::HandleNewActionUpdate()
{
	cScenarioExpImitateStep::HandleNewActionUpdate();

	SyncKinChar();
}

void cScenarioExpHikeGeneral::HandleFallUpdate()
{
	cScenarioExpImitateStep::HandleFallUpdate();

}


tVector cScenarioExpHikeGeneral::CalcTargetPosDefault()
{
	const double max_dist = GetRandTargetMaxDist();

	tVector target_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();
	target_pos[0] = root_pos[0] + mRand.RandDouble(-max_dist, max_dist);
	target_pos[2] = root_pos[2] + mRand.RandDouble(-max_dist, max_dist);

	return target_pos;
}

int cScenarioExpHikeGeneral::GetTargetPosTrail3dForwardSegs() const
{
	return 10;
}
