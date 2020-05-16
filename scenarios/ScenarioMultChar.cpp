#include "ScenarioMultChar.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/GroundDynamicCharacters3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/WaypointController.h"

cScenarioMultChar::tAgentData::tAgentData()
{
	reachedTarget = false;
}

cScenarioMultChar::cScenarioMultChar() :
	cScenarioExpHike(),
	cScenarioImitateStepEval()
{
	EnableTargetPos(false);
	EnableRandTargetPos(true);
	mNumChars=0;
	mSpawnRadius=10.0;
	mRandTargetBound = 10.0;
	mUseSimpleReward = false;
	mUseSimpleDistanceReward = false;
	mUseRepulsiveReward = false;
	mRewardEffortQuality = false;
	mRandomizeInititalRotation = false;
	mCreateNewGoals = false;
	mReachTargetBonus = 200;
	mUsePursuitConfig = false;
	mTargetRewardWeight = 1;
}

cScenarioMultChar::~cScenarioMultChar()
{
}

void cScenarioMultChar::Init()
{
	cScenarioExpHike::Init();
	cScenarioImitateStepEval::Init();
	BuildChars();
	SetupGround();
	agentDatas.push_back(tAgentData());
	// agentDatas[0].reachedTarget = false;
	for (size_t a =0; a < mChars.size(); a++)
	{
		mPrevCOMs.push_back(tVector(0,0,0,0));
		mPrevTimes.push_back(0);
		agentDatas.push_back(tAgentData());
		// agentDatas[a].reachedTarget = false;
	}
	EnableTargetPos(false);
}

void cScenarioMultChar::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpHike::ParseMiscArgs(parser);  // BRANDON WORKING HERE ON PARSIN GINPUTS
	
	parser->ParseInt("num_characters", mNumChars);
	parser->ParseDouble("spawn_radius", mInitSpawnRadius);
	parser->ParseDouble("rand_target_bound", mInitRandTargetBound);
	parser->ParseDouble("density_range_percent", mDensityModulationRange);
	parser->ParseBool("reward_effort_quality", mRewardEffortQuality);
	parser->ParseBool("randomize_initial_rotation", mRandomizeInititalRotation);
	parser->ParseBool("create_new_goals", mCreateNewGoals);

	mSpawnRadius = mInitSpawnRadius;
	mRandTargetBound = mInitRandTargetBound;
	parser->ParseBool("use_simple_reward", mUseSimpleReward);
	parser->ParseBool("use_repulsive_reward", mUseRepulsiveReward);
	parser->ParseBool("use_persuit_config", mUsePursuitConfig);
	parser->ParseBool("use_simple_distance_reward", mUseSimpleDistanceReward);
}

const std::shared_ptr<cSimCharacter>& cScenarioMultChar::GetSpaceCharacter() const
{
	return mChar1;
}

std::string cScenarioMultChar::GetName() const
{
	return "Scenario similar to Hike but with multiple characters";
}

void cScenarioMultChar::BuildChars()
{
	// Initialize all of the characters
	for(int i = 0; i < mNumChars; i++) 
	{
		std::shared_ptr<cSimCharacter> newChar;
		CreateCharacter(eCharBiped3D, newChar);

		cSimCharacter::tParams char_params = mCharParams;
		char_params.mInitPos = GetDefaultCharPos();
		// char_params.mCharFile = this->getRelativeFilePath() + "data/characters/biped3d_mocap1.txt";
		// char_params.mStateFile = this->getRelativeFilePath() + "data/states/biped3d_sim_walk_state.txt";
		// char_params.mGroundSampleRes3d = 32;
		// char_params.mViewDist = 5;

		bool succ = newChar->Init(mWorld, char_params);
		if (succ)
		{
			newChar->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
			InitCharacterPos(newChar);

			std::shared_ptr<cCharController> ctrl;
			cTerrainRLCtrlFactory::tCtrlParams ctrl_params(mCtrlParams);
			ctrl_params.mCharCtrl = mCtrlParams.mCharCtrl;
			// ctrl_params.mCtrlParamFile = this->getRelativeFilePath() + "data/characters/biped3d_mocap1.txt";
			ctrl_params.mChar = newChar;
			ctrl_params.mGravity = GetGravity();
			ctrl_params.mGround = mGround;
			ctrl_params.mEnableSymmetricLLC = true;

			ctrl_params.mCtQueryRate = 0;

			// The following are not necessary under the python learning framework
			ctrl_params.mNetFiles = mCtrlParams.mNetFiles;
			// ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorSolver] = this->getRelativeFilePath() + "data/policies/biped3d/nets/biped3d_sym_step_dphase_actor_solver.prototxt";
			// ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor] = this->getRelativeFilePath() + "data/policies/biped3d/nets/biped3d_sym_step_dphase_actor_net.prototxt";
			// ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = this->getRelativeFilePath() + "data/policies/biped3d/models/biped3d_sym_step_dphase_model1.h5";

			ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorSolver] = "";
			ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor] = "";
			ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = "";

			/// This is only helpful if you want controllers of different types...
			bool succ = cTerrainRLCtrlFactory::BuildController(ctrl_params, ctrl);
			/// Can use this if you just want copies of the default controller
			// bool succ = cTerrainRLCtrlFactory::BuildController(mCtrlParams, ctrl);

			if (succ && ctrl != nullptr)
			{
				newChar->SetController(ctrl);
				std::cout << "Created another character controller" << std::endl;
			}
			else
			{
				std::cout << "Failed creating another character controller" << std::endl;
			}
			/*
			if(params.mGroundSampleRes3d >= 0)
			{
				ctrl->SetGroundSampleRes(params.mGroundSampleRes3d);
			}
		    if(params.mViewDist >= 0)
		    {
		        ctrl->SetViewDist(params.mViewDist);
		    }
		    */
		}

		GenerateInitialTransform(newChar);
		newChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(newChar));

		previousPositions.push_back(newChar->GetRootPos());
		masses.push_back(ComputeTotalCharacterMass(newChar));
		mChars.push_back(newChar);
		rewards.push_back(0.0);
	}	


	//////////////////////////////////////////////////////////////
	GenerateInitialTransform(mChar);
	previousPosition = mChar->GetRootPos();
	mass = ComputeTotalCharacterMass(mChar);
	mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));
}

void cScenarioMultChar::UpdateCharacter(double time_step)
{
	cScenarioExpHike::UpdateCharacter(time_step);

	// Update default mChar Target if reached
	if ( mCreateNewGoals && (mChar->GetCurrentGroundTarget() - mChar->GetRootPos()).squaredNorm() < 2.0 ) // May be problematic because target is on ground plane
	{
		agentDatas[0].reachedTarget = true;
		mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));
	}

	for(size_t a = 0; a < mChars.size(); a++)
	{
    	mChars[a]->Update(time_step);

    	// Update target if reached for this char
    	if ( mCreateNewGoals && (mChars[a]->GetCurrentGroundTarget() - mChars[a]->GetRootPos()).squaredNorm() < 2.0 ) // May be problematic because target is ground plane
		{
    		agentDatas[a+1].reachedTarget = true;
    		mChars[a]->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChars[a]));
		}
    	if (mUsePursuitConfig)
    	{
    		mChars[a]->SetCurrentGroundTarget(mChar->GetRootPos());
    	}
	}
}

/*
bool cScenarioMultChar::EnableLLCFeedbackReward() const
{
	return true;
}
*/

double cScenarioMultChar::CalcReward()
{
	const double target_speed = 1.0;

	const tVector& prev_com = mPrevCOM;
	// double time_elapsed = mTime - mPrevTime;
	double time_elapsed = 0.5;
	double vel_w = 0.8;
	double step_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double heading_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double LLC_reward_w = 0 * ((EnableLLCFeedbackReward()) ? 0.2 : 0);
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
	double worst_reward = 0;

	if (time_elapsed > 0)
	{
		bool fallen = HasFallen();
		// std::cout << "default char has fallen: " << fallen << std::endl;
		if (!fallen)
		{
			tVector curr_com = mChar->CalcCOM();
			tVector com_delta = curr_com - prev_com;
			tVector target_delta = mChar->GetCurrentGroundTarget() - prev_com;
			com_delta[1] = 0;
			target_delta[1] = 0;
			tVector target_dir = target_delta.normalized();



			double avg_vel = target_dir.dot(com_delta);
			avg_vel /= time_elapsed;
			double vel_err = std::min(0.0, avg_vel - target_speed);
			// double target_dist_threshold = GetTargetResetDist();
			double target_dist_threshold = 0.35;
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

			if (mUseSimpleReward)
			{
				tVector groundVelocity = mChar->CalcCOMVel();
				groundVelocity[1] = 0.0;
				tVector groundPosition = mChar->GetRootPos();
				groundPosition[1] = 0.0;
				tVector groundTargetNormal = (mChar->GetCurrentGroundTarget() - groundPosition).normalized();
				double targetRewardExponent = groundTargetNormal.transpose() * groundVelocity - 1.0;

				reward = reward + mTargetRewardWeight * std::exp( -std::pow( std::min(0.0, targetRewardExponent), 2.0 ) );
				if (agentDatas[0].reachedTarget)
				{
					reward = reward + mReachTargetBonus;
					agentDatas[0].reachedTarget = false;
				}
			}
			if (mUseSimpleDistanceReward)
			{
				tVector groundPosition = mChar->GetRootPos();
				groundPosition[1] = 0.0;
				double groundTargetNormal = (mChar->GetCurrentGroundTarget() - groundPosition).norm();
				// double targetRewardExponent = groundTargetNormal.transpose() * groundVelocity - 1.0;

				reward = reward + mTargetRewardWeight * std::exp( std::pow( groundTargetNormal, 2.0 ) * -0.5 );
				// std::cout << "ball dist reward " << reward << std::endl;
				if (agentDatas[0].reachedTarget)
				{
					reward = reward + mReachTargetBonus;
					agentDatas[0].reachedTarget = false;
				}
			}

			if (mUseRepulsiveReward)
			{
				// Check repulsive reward with all other agents
				for (size_t ag = 0; ag < this->mChars.size(); ag++)
				{
					std::shared_ptr<cSimCharacter> other_agent = this->mChars[ag];
					double dist__2 = (mChar->GetRootPos() - other_agent->GetRootPos()).squaredNorm();
					if ( dist__2 < 3.0)
					{
						tVector groundVelocity = mChar->CalcCOMVel().normalized();
						tVector groundVelocity2 = other_agent->CalcCOMVel().normalized();
						double relativeAngle = groundVelocity.dot(groundVelocity2) - 1.0;
						reward = reward - (0.5 + ((3.0 - dist__2)*relativeAngle));
						// std::cout << "close collision: " << dist__2 << " at angle: " << relativeAngle << " penalty: " << reward << std::endl;
					}
					worst_reward =- 1;

				}
				/// check collisions with obstacles
				if (true)
				{
					// tVector char_aabb_min;
					// tVector char_aabb_max;
					// mChar->CalcAABB(char_aabb_min, char_aabb_max);
					tVector curr_com_h = curr_com;
					tVector unit_v = tVector(1.0,1.0,1.0,0);
					curr_com_h[1] = 0;
					for (int i = 0; i < mGround->GetNumObstacles(); ++i)
					{
						const auto& obj = mGround->GetObj(i);
						tVector aabb_min;
						tVector aabb_max;
						obj.CalcAABB(aabb_min, aabb_max);
						aabb_min = aabb_min - unit_v;
						aabb_max = aabb_max + unit_v;

						if (cMathUtil::ContainsAABB(curr_com_h, aabb_min, aabb_max))
						{
							reward = reward - (1.0);
							break;
						}
					}
				}
			}

			// Effort Quality metric
			if (mRewardEffortQuality) {
				double L = (mChar->GetRootPos() - previousPosition).norm();
				double optimalEffort = 2.0 * mass * L * 1.33;	// 2.0 * m * L * sqrt(e_s * e_w)
				double actual_effort = mass * (2.23 + (1.26 * mChar->CalcCOMVel().squaredNorm())) * time_elapsed;  // m * INT (e_s + e_w * |v|^2)dt
				reward += optimalEffort / actual_effort;
			}

			// Store previous position
			previousPosition = mChar->GetRootPos();
		}
		else
		{ /// If fallen get bad reward
			reward = worst_reward - 2;
		}
	}
	return reward;
}

double cScenarioMultChar::calcRewardForAgent(size_t agent)
{
	/*
	if (agent == 0)
	{
		return this->CalcReward();
	}
	*/

	std::shared_ptr<cSimCharacter> char_ = this->mChars[agent];
	tVector _TargetPos = char_->GetCurrentGroundTarget();
	tVector _PrevCOM = mPrevCOMs[agent];
	// std::cout << "_PrevCOM: " << _PrevCOM << std::endl;
	auto ctrl_ = std::dynamic_pointer_cast<cWaypointController>(char_->GetController());
	// std::dynamic_pointer_cast<cWaypointController>
	auto _StepPlan = ctrl_->GetStepPlan();
	/*
	if ( ctrl_ != nullptr )
	{
		auto _StepPlan = ctrl_->GetStepPlan();
	}
	*/
	const double target_speed = mTargetSpeed;

	const tVector& prev_com = _PrevCOM;
	// double time_elapsed = mTime - mPrevTime;
	// double time_elapsed = mTime - mPrevTimes[agent];
	double time_elapsed = 0.5;
	double vel_w = 0.8;
	double step_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double heading_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double LLC_reward_w = 0 * ((EnableLLCFeedbackReward()) ? 0.2 : 0);
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
		// char_->IsInContact();
		bool fallen = char_->HasFallen();
		if (!fallen)
		{
			tVector curr_com = char_->CalcCOM();
			// std::cout << "curr_com: " << curr_com.transpose() << std::endl;
			tVector com_delta = curr_com - prev_com;
			// std::cout << "com_delta: " << com_delta.transpose() << std::endl;
			tVector target_delta = _TargetPos - prev_com;
			// std::cout << "target_delta: " << target_delta.transpose() << std::endl;
			com_delta[1] = 0;
			target_delta[1] = 0;
			tVector target_dir = target_delta.normalized();
			// std::cout << "target_dir: " << target_dir.transpose() << std::endl;

			double avg_vel = target_dir.dot(com_delta);
			// std::cout << "avg_vel: " << avg_vel << std::endl;
			avg_vel /= time_elapsed;
			double vel_err = std::min(0.0, avg_vel - target_speed);
			double target_dist_threshold = GetTargetResetDist();
			if (target_delta.squaredNorm() < target_dist_threshold * target_dist_threshold)
			{
				vel_err = 0;
			}
			vel_err *= vel_err;
			// std::cout << "vel_err: " << vel_err << std::endl;

			eStance stance = GetStance();
			int stance_foot_id = GetStanceFootJoint(stance);
			tVector stance_foot_pos = char_->CalcJointPos(stance_foot_id);
			const tVector& step_pos = _StepPlan.mStepPos0;
			tVector step_delta = step_pos - stance_foot_pos;
			double step_err = step_delta.squaredNorm();
			// std::cout << "step_err: " << step_err << std::endl;

			double heading = char_->CalcHeading();
			double tar_heading = _StepPlan.mRootHeading;
			double heading_err = std::abs(tar_heading - heading);
			heading_err = std::min(2 * M_PI - heading_err, heading_err);
			// std::cout << "heading_err: " << heading_err << std::endl;

			double vel_reward = std::exp(-vel_scale * vel_err);
			//vel_reward = (avg_vel > 0) ? vel_reward : 0;
			double step_reward = exp(-step_scale * step_err);
			double heading_reward = 0.5 * (std::cos(heading_err) + 1);
			heading_reward = std::pow(heading_reward, 4);

			// std::cout << "vel_reward: " << vel_reward <<
			// 		" step_reward: " << step_reward <<
			// 		" heading_reward: " << heading_reward << std::endl;

			reward = vel_w * vel_reward + step_w * step_reward + heading_w * heading_reward
					+ fixed_w;
			// std::cout << "agent: " << agent << " reward: " << reward << std::endl;
			// Path following metric
			if (mUseSimpleReward)
			{
				tVector groundVelocity = char_->CalcCOMVel();
				groundVelocity[1] = 0.0;
				tVector groundPosition = char_->GetRootPos();
				groundPosition[1] = 0.0;
				tVector groundTargetNormal = (char_->GetCurrentGroundTarget() - groundPosition).normalized();
				double targetRewardExponent = groundTargetNormal.transpose() * groundVelocity - 1;

				reward = reward + mTargetRewardWeight * std::exp( -std::pow( std::min(0.0, targetRewardExponent), 2.0 ) );
				if (agentDatas[agent+1].reachedTarget)
				{
					reward = reward + mReachTargetBonus;
					// std::cout << "Agent reached target: " << reward  << std::endl;
					agentDatas[agent+1].reachedTarget = false;
				}
			} else if (mUseSimpleDistanceReward)
			{
				tVector groundPosition = char_->GetRootPos();
				groundPosition[1] = 0.0;
				double groundTargetNormal = (char_->GetCurrentGroundTarget() - groundPosition).norm();
				// double targetRewardExponent = groundTargetNormal.transpose() * groundVelocity - 1.0;

				reward = reward + mTargetRewardWeight * std::exp( std::pow( groundTargetNormal, 2.0 ) * -0.8 );
				if (agentDatas[agent+1].reachedTarget)
				{
					reward = reward + mReachTargetBonus;
					agentDatas[agent+1].reachedTarget = false;
				}
			}

			if (mUseRepulsiveReward)
			{
				// Check repulsive reward with all other agents
				for (size_t ag = 0; ag < this->mChars.size(); ag++)
				{
					std::shared_ptr<cSimCharacter> other_agent = this->mChars[ag];
					double dist__s = (char_->GetRootPos() - other_agent->GetRootPos()).squaredNorm();
					if ( dist__s < 3.0
							&& (agent  != ag ))
					{
						tVector groundVelocity = char_->CalcCOMVel().normalized();
						tVector groundVelocity2 = other_agent->CalcCOMVel().normalized();
						double relativeAngle = groundVelocity.dot(groundVelocity2) - 1.0;
						reward = reward - (0.5 + ((3.0 - dist__s)*relativeAngle));
					}

				}
				double dist__2 = (char_->GetRootPos() - mChar->GetRootPos()).squaredNorm();
				if ( dist__2 < 3.0)
				{
					tVector groundVelocity = mChar->CalcCOMVel().normalized();
					tVector groundVelocity2 = char_->CalcCOMVel().normalized();
					double relativeAngle = groundVelocity.dot(groundVelocity2) - 1.0;
					reward = reward - (0.5 + ((3.0 - dist__2)*relativeAngle));
				}
				/// check collisions with obstacles
				if (true)
				{
					// tVector char_aabb_min;
					// tVector char_aabb_max;
					// mChar->CalcAABB(char_aabb_min, char_aabb_max);
					tVector curr_com_h = curr_com;
					tVector unit_v = tVector(1.0,1.0,1.0,0);
					curr_com_h[1] = 0;
					for (int i = 0; i < mGround->GetNumObstacles(); ++i)
					{
						const auto& obj = mGround->GetObj(i);
						tVector aabb_min;
						tVector aabb_max;
						obj.CalcAABB(aabb_min, aabb_max);
						aabb_min = aabb_min - unit_v;
						aabb_max = aabb_max + unit_v;

						if (cMathUtil::ContainsAABB(curr_com_h, aabb_min, aabb_max))
						{
							reward = reward - (1.0);
							// std::cout << "found obstacle penalty" << std::endl;
							break;
						}
					}
				}
			}

			// Effort Quality metric
			if (mRewardEffortQuality) {
				double L = (char_->GetRootPos() - this->previousPositions[agent]).norm();
				double optimalEffort = 2.0 * this->masses[agent] * L * 1.33;	// 2.0 * m * L * sqrt(e_s * e_w)
				double actual_effort = this->masses[agent] * (2.23 + (1.26 * char_->CalcCOMVel().squaredNorm())) * time_elapsed;  // m * INT (e_s + e_w * |v|^2)dt
				reward += optimalEffort / actual_effort;	// Store reward
			}

			// Store previous position
			this->previousPositions[agent] = char_->GetRootPos();
		}
		else
		{
			reward = -1.0;
		}
	}
	return reward;
}

int cScenarioMultChar::GetCharacterCount ()
{
	return mNumChars;
}

double cScenarioMultChar::ComputeTotalCharacterMass(std::shared_ptr<cSimCharacter>& character)
{
	double total_mass = 0.0;

	for (int i = 0; i < character->GetNumBodyParts(); ++i)
	{
		if (character->IsValidBodyPart(i))
		{
			const auto& part = character->GetBodyPart(i);
			double mass = part->GetMass();
			total_mass += mass;
		}
	}

	return total_mass;
}

void cScenarioMultChar::GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character)
{
	// Find a clear ground plane point to place the character
	// do
	{
		character->Reset();

		// Handle position
		tVector root_pos1 = character->GetRootPos();
		root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0.82, mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
		
		bool freeSpace = true;

		// Find any intersections with previously placed characters
		// Note we want to avoid any intersections with any characters, thus the inner loop which checks all characters
		// given the current generated starting position, and generates a new one if any intersection is found
		/*
		do
		{
			for(std::vector<std::shared_ptr<cSimCharacter>>::iterator it = mChars.begin(); it != mChars.end(); ++it) 
			{
				// if the current character we are looking at is not in the default position (ie has been placed)
				if((GetDefaultCharPos() - (*it)->GetRootPos()).norm() > 0.000001 )
				{
					// and is within our bounds then this is not a free space, also check default character here
					if ( (root_pos1 - (*it)->GetRootPos()).norm() < 1.0 || (root_pos1 - mChar->GetRootPos()).norm() < 1.0) 
					{
						root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 1.0, mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
						freeSpace = false;
						break;
					} else freeSpace = true;
				}
			}
		} while (!freeSpace);
		*/

		// ResolveCharGroundIntersect(character);

		// Handle rotation
		if (mRandomizeInititalRotation)
		{
			tVector rotation_axis;
			double rotation_angle;

			rotation_angle = mRand.RandDouble(-M_PI, M_PI);
			rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);

			tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, rotation_angle);
			
			// TODO: Need to fix this, character initial link velocities need to be lined up with this rotation
			// character->SetRootRotation(new_rotation);

			// Sets rotation and position
			character->SetRootTransform(root_pos1, new_rotation);
		} else {
			// Set position
			character->SetRootPos(root_pos1);
			character->SetRootPos0(root_pos1);
		}

	}
	// while (character->GetRootPos()[1] - 0 > 0.82); //  Ensures character finds a spot on the flat ground
}

void cScenarioMultChar::Reset()
{
	cScenarioExpHike::Reset();

	GenerateInitialTransform(mChar);
	mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));
	previousPosition = mChar->GetRootPos();


	// Find random positions somewhere on the open ground for the rest of the characters
	std::vector<std::shared_ptr<cSimCharacter>>::iterator it;
	std::vector<tVector>::iterator pos;

	for(it = mChars.begin(), pos = previousPositions.begin(); 
		it != mChars.end() && pos != previousPositions.end(); 
		++it, ++pos) 
	{
		GenerateInitialTransform((*it));
		(*pos) = (*it)->GetRootPos();
		(*it)->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D((*it)));
		// (*it)->SetRootRotation(mChar->GetRootRotation());
	}

	for (size_t a =0; a < mChars.size(); a++)
	{
		mPrevCOMs[a] = mChars[a]->CalcCOM();
		mPrevTimes[a] = mTime;
	}
	EnableTargetPos(false);
}

void cScenarioMultChar::SyncCharacters()
{
	cScenarioExpHike::SyncCharacters();

	const Eigen::VectorXd& pose = mKinChar->GetPose();
	const Eigen::VectorXd& vel = mKinChar->GetVel();

	for (size_t a =0; a < mChars.size(); a++)
	{
		mChars[a]->SetPose(pose);
		mChars[a]->SetVel(vel);

		const auto& ctrl = mChars[a]->GetController();
		auto phase_ctrl = std::dynamic_pointer_cast<cCtPhaseController>(ctrl);
		if (phase_ctrl != nullptr)
		{
			double kin_time = mKinChar->GetTime();
			phase_ctrl->SetTime(kin_time);
		}

		auto waypoint_ctrl = std::dynamic_pointer_cast<cWaypointController>(ctrl);
		if (waypoint_ctrl != nullptr)
		{
			double kin_time = mKinChar->GetTime();
			waypoint_ctrl->SetTime(kin_time);
		}
	}

}

tVector cScenarioMultChar::CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character)
{
	tVector target_pos = tVector::Zero();
	double rand_target_bound = mRandTargetBound;
	double height = 0;
	bool valid_sample;
	tVector root_pos = character->GetRootPos();
	double minX = root_pos[0] - rand_target_bound;
	double maxX = root_pos[0] + rand_target_bound;
	double minZ = root_pos[2] - rand_target_bound;
	double maxZ = root_pos[2] + rand_target_bound;
	do
	{
		target_pos = tVector(mRand.RandDouble(minX, maxX), 0, mRand.RandDouble(minZ, maxZ), 0);
		height = mGround->SampleHeight(target_pos, valid_sample);	
	} while ( height > 0 );

	return target_pos;
}

void cScenarioMultChar::SetupGround()
{
	auto dynamic_obstacles_characters = std::dynamic_pointer_cast<cGroundObstaclesDynamicCharacters3D>(mGround);
	if (dynamic_obstacles_characters != nullptr)
	{
		dynamic_obstacles_characters->SetChar(mChar);
		dynamic_obstacles_characters->SetChars(mChars);
		// std::shared_ptr<cScenario> _scenario = std::shared_ptr<cScenario>(this);
		// std::cout << "_scenario: " << _scenario << std::endl;
		// dynamic_obstacles_characters->setScenario(_scenario);
	}
	auto dynamic_obstacles = std::dynamic_pointer_cast<cGroundDynamicObstacles3D>(mGround);
	if (dynamic_obstacles != nullptr)
	{
		dynamic_obstacles->SetChar(mChar);
	}
	auto dynamic_characters = std::dynamic_pointer_cast<cGroundDynamicCharacters3D>(mGround);
	if (dynamic_characters != nullptr)
	{
		dynamic_characters->SetChar(mChar);
		dynamic_characters->SetChars(mChars);
		/// dynamic_obstacles_characters->setScenario(std::shared_ptr<cScenario>(this));
	}
}

void cScenarioMultChar::HandleNewActionUpdate()
{
	cScenarioExpHike::HandleNewActionUpdate();
	for (size_t a =0; a < mChars.size(); a++)
	{
		mPrevCOMs[a] = mChars[a]->CalcCOM();
		mPrevTimes[a] = mTime;
	}
}
