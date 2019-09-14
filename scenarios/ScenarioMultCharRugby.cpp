#include "ScenarioMultCharRugby.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/WaypointController.h"
#include "sim/SimSphere.h"

const double gMaxTargetDist = 10;
const double gMaxBallDist = 10;

const double gComBallDistThreshold = 2;

const double gBallRadius = 0.2;
const tVector gBallColor = tVector(0.8, 0.8, 0.8, 1);

cScenarioMultCharRugby::cScenarioMultCharRugby() :
	cScenarioMultChar()
{
	EnableTargetPos(true);
	EnableRandTargetPos(true);
	mNumChars=0;
	mSpawnRadius=10.0;
	mRandTargetBound = 10.0;
	// mNumBallSpawns=1;
}

cScenarioMultCharRugby::~cScenarioMultCharRugby()
{
}

void cScenarioMultCharRugby::Init()
{
	cScenarioMultChar::Init();
}


std::string cScenarioMultCharRugby::GetName() const
{
	return "Scenario with multiple characters in rugby formation";
}

void cScenarioMultCharRugby::UpdateCharacter(double time_step)
{
	cScenarioExpHike::UpdateCharacter(time_step);
	int num_agents = 1 +  mChars.size();
	// Update default mChar Target if reached
	mChar->SetCurrentGroundTarget(GetBallPos());
	if ( mCreateNewGoals && (mChar->GetCurrentGroundTarget()[0] > 5.0) ) // May be problematic because target is on ground plane
	{
		agentDatas[0].reachedTarget = true;
		mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));
	}

	for(size_t a = 0; a < (mChars.size() - (num_agents/2)); a++)
	{
    	mChars[a]->Update(time_step);

		mChars[a]->SetCurrentGroundTarget(GetBallPos());
    	// Update target if reached for this char
    	if ( mCreateNewGoals && (mChars[a]->GetCurrentGroundTarget()[0] > 5.0) ) // May be problematic because target is ground plane
		{
    		agentDatas[a+1].reachedTarget = true;
    		mChars[a]->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChars[a]));
		}
	}
	for(size_t a = (num_agents/2)-1; a < mChars.size(); a++)
	{
		mChars[a]->Update(time_step);

		mChars[a]->SetCurrentGroundTarget(GetBallPos());
		// Update target if reached for this char
		if ( mCreateNewGoals && (mChars[a]->GetCurrentGroundTarget()[0] < -5.0) ) // May be problematic because target is ground plane
		{
			agentDatas[a+1].reachedTarget = true;
			mChars[a]->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChars[a]));
		}
	}
	if (GetBallPos()[0] < -5 || (GetBallPos()[0] > 5.0))
	{
		// int handle = GetTargetBallHandle();
		// RemoveObj(handle);
		SetBallPos(tVector(0,1,0,0));
	}
}


void cScenarioMultCharRugby::BuildChars()
{
	// Initialize all of the characters
	BuildBall();

	double rand_rot = mRand.RandDouble(-M_PI, M_PI);
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

		GenerateInitialTransform(newChar, i+1);
		newChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(newChar));

		previousPositions.push_back(newChar->GetRootPos());
		masses.push_back(ComputeTotalCharacterMass(newChar));
		mChars.push_back(newChar);
		rewards.push_back(0.0);
	}	


	//////////////////////////////////////////////////////////////
	GenerateInitialTransform(mChar, 0);
	previousPosition = mChar->GetRootPos();
	mass = ComputeTotalCharacterMass(mChar);
	mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));

}


void cScenarioMultCharRugby::GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t a)
{
	// Find a clear ground plane point to place the character
	// do
	{
		character->Reset();

		// Handle position
		tVector root_pos1 = character->GetRootPos();
		root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, mSpawnRadius), root_pos1[1], mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
		bool freeSpace = true;
		// std::cout << ((mNumChars + 1)/2) << std::endl;
		if (a < ((mNumChars + 1)/2))
		{
			root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, -1.0), root_pos1[1], mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
		}
		else
		{
			root_pos1 = tVector(mRand.RandDouble(1, mSpawnRadius), root_pos1[1], mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
		}
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
		tVector rotation_axis;
		double rotation_angle;

		rotation_angle = mRand.RandDouble(-M_PI, M_PI);
		rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);

		tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, rotation_angle);

		// TODO: Need to fix this, character initial link velocities need to be lined up with this rotation
		// character->SetRootRotation(new_rotation);

		// Sets rotation and position
		character->SetRootTransform(root_pos1, new_rotation);

	}
	// while (character->GetRootPos()[1] - 0 > 0.82); //  Ensures character finds a spot on the flat ground
}

void cScenarioMultCharRugby::Reset()
{
	cScenarioExpHike::Reset();

	SetBallPos(tVector(0,0.5,0,0));
	GenerateInitialTransform(mChar, 0);
	mChar->SetCurrentGroundTarget(GetBallPos());
	previousPosition = mChar->GetRootPos();


	// Find random positions somewhere on the open ground for the rest of the characters
	// std::vector<std::shared_ptr<cSimCharacter>>::iterator it;
	// std::vector<tVector>::iterator pos;

	for (size_t it = 0; it <  mChars.size() ; it++)
	{
		GenerateInitialTransform((mChars[it]), it+1);
		// pos = mChars[it]->GetRootPos();
		mChars[it]->SetCurrentGroundTarget(GetBallPos());
		// (*it)->SetRootRotation(mChar->GetRootRotation());
	}

	for (size_t a =0; a < mChars.size(); a++)
	{
		mPrevCOMs[a] = mChars[a]->CalcCOM();
		mPrevTimes[a] = mTime;
	}
	EnableTargetPos(false);
}

tVector cScenarioMultCharRugby::CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character)
{

	tVector rotation_axis;
	double rotation_angle;
	double circle_rad = mSpawnRadius + mRand.RandDouble(-2, 1);
	tVector origin = tVector(0,0,0,0);
	tVector dist = tVector(1.0,0,0,0);
	rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);
	double portion = (M_PI * 2) / (mNumChars + 1);
	/// To add more noise to the initial circle positions
	double rand_circle_rot = mRand.RandDouble(-M_PI, M_PI);;
	tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, rand_circle_rot);
	tVector target_pos = cMathUtil::QuatRotVec(new_rotation, dist * circle_rad);
	// std::cout << "new target position: " << target_pos << std::endl;

	return target_pos;
}


////////////////////////////////////////////////////////////////////////////
////// Soccer -> Rugby ball handling


void cScenarioMultCharRugby::ResetParams()
{
	cScenarioExpHike::ResetParams();
	// mPrevBallPos.setZero();
}

bool cScenarioMultCharRugby::CheckResetTarget() const
{
	bool reset_target = (mRandTargetPosTimer <= 0);
	return reset_target;
}

void cScenarioMultCharRugby::ClearObjs()
{
	cScenarioExpHike::ClearObjs();
	// mBallObjHandles.clear();
}

void cScenarioMultCharRugby::HandleNewActionUpdate()
{
	cScenarioMultChar::HandleNewActionUpdate();

	// mPrevBallPos = GetBallPos();
}
/*
bool cScenarioMultCharRugby::EndEpisode() const
{
	bool is_end = cScenarioExpHike::EndEpisode();
	int num_balls = GetNumBalls();
	is_end |= (num_balls < 1);
	return is_end;
}

double cScenarioMultCharRugby::GetRandTargetMaxDist() const
{
	return gMaxTargetDist;
}

double cScenarioMultCharRugby::GetRandBallMaxDist() const
{
	return gMaxBallDist;
}

void cScenarioMultCharRugby::BuildBalls()
{
	for (int i = 0; i < mNumBallSpawns; ++i)
	{
		int curr_handle = BuildBall();
		mBallObjHandles.push_back(curr_handle);
	}
	// UpdateTargetBall();
}*/

int cScenarioMultCharRugby::BuildBall()
{
	const double r = gBallRadius;
	const double mass = 0.1;
	const double linear_damping = 0.5;
	const double angular_damping = 0.5;
	const double friction = 0.5;

	cSimSphere::tParams params;
	params.mRadius = r;
	params.mPos = tVector(1, 1, 1, 0);
	params.mVel = tVector::Zero();
	params.mFriction = friction;
	params.mMass = mass;
	auto ball_ = std::shared_ptr<cSimSphere>(new cSimSphere());

	ball_->Init(mWorld, params);
	ball_->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);
	ball_->SetDamping(linear_damping, angular_damping);
	ball_->DisableDeactivation();
	this->ball = ball_;

	tObjEntry obj_entry;
	obj_entry.mObj = ball;
	obj_entry.mEndTime = std::numeric_limits<double>::infinity();
	obj_entry.mColor = gBallColor;

	// int ball_handle = AddObj(obj_entry);
	// ballObjHandle = ball_handle;
	return 0;
}


const std::shared_ptr<cSimObj>& cScenarioMultCharRugby::GetBall() const
{
	return this->ball;
}
/*
const std::shared_ptr<cSimObj>& cScenarioMultCharRugby::GetBall(int ball_handle) const
{
	assert(ball_handle != gInvalidIdx);
	const tObjEntry& entry = mObjs[ball_handle];
	return entry.mObj;
}
*/

tVector cScenarioMultCharRugby::GetBallPos() const
{
	return this->ball->GetPos();
}
/*
tVector cScenarioMultCharRugby::GetBallPos(int ball_handle) const
{
	const auto& ball = GetBall(ball_handle);
	return ball->GetPos();
}*/

/*
tVector cScenarioMultCharRugby::CalcTargetPosDefault()
{
	//return cScenarioExpHike::CalcTargetPosDefault();
	const double max_dist = GetRandTargetMaxDist();
	const double min_dist = 0;

	tVector rand_pos = tVector::Zero();
	tVector ball_pos = tVector::Zero();
	int ball_handle = GetTargetBallHandle();
	if (ball_handle != gInvalidIdx)
	{
		ball_pos = GetBallPos(ball_handle);
	}

	double r = mRand.RandDouble(min_dist, max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	rand_pos[0] = ball_pos[0] + r * std::cos(theta);
	rand_pos[2] = ball_pos[2] + r * std::sin(theta);

	return rand_pos;
}

void cScenarioMultCharRugby::UpdateBallPos(double time_elapsed)
{
	std::cout << "update ball" << std::endl;
	int ball_handle = GetTargetBallHandle();

#if defined(ENABLE_PHANTOM_BALL)
	const double dist_threshold = 0.2;
	tVector root_pos = mChar->GetRootPos();
	tVector ball_pos = GetBallPos();
	root_pos[1] = 0;
	ball_pos[1] = 0;

	double dist_sq = (ball_pos - root_pos).squaredNorm();
	if (dist_sq < dist_threshold)
	{
		tVector com_vel = mChar->CalcCOMVel();
		com_vel[1] = 0;
		tVector com_vel_dir = com_vel.normalized();
		ball_pos += 0.5 * com_vel_dir;
		SetBallPos(ball_pos);
	}
#endif
}

void cScenarioMultCharRugby::ResetBallPosAll()
{
	for (int i = 0; i < GetNumBalls(); ++i)
	{
		ResetBallPos(mBallObjHandles[i]);
	}

	UpdateTargetBall();
	ResetBallTimer();
}

void cScenarioMultCharRugby::ResetBallPos(int ball_handle)
{
	const double min_dist = 0.7;
	const double max_dist = GetRandBallMaxDist();
	assert(min_dist <= max_dist);

	tVector rand_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();
	double r = mRand.RandDouble(min_dist, max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	rand_pos[0] = root_pos[0] + r * std::cos(theta);
	rand_pos[2] = root_pos[2] + r * std::sin(theta);

	SetBallPos(ball_handle, rand_pos);
}
*/
void cScenarioMultCharRugby::SetBallPos(const tVector& pos)
{
	ball->SetPos(pos);
	ball->SetRotation(tQuaternion::Identity());
	ball->SetLinearVelocity(tVector::Zero());
	// ball->SetLinearVelocity(tVector(15.0,0,0,0));
	ball->SetAngularVelocity(tVector::Zero());
}
/*
int cScenarioMultCharRugby::GetNumBalls() const
{
	return static_cast<int>(mBallObjHandles.size());
}


void cScenarioMultCharRugby::RemoveObj(int handle)
{
	int num_objs = GetNumObjs();
	cScenarioExpHike::RemoveObj(handle);

	// update ball handle since removing objects might have caused it to change
	auto end_ball = std::find(mBallObjHandles.begin(), mBallObjHandles.end(), num_objs - 1);
	if (end_ball != mBallObjHandles.end())
	{
		*end_ball = handle;

		if (handle == GetTargetBallHandle())
		{
			mBallObjHandles.pop_back();
			UpdateTargetBall();
		}
	}
}

void cScenarioMultCharRugby::SetBallPos(int ball_handle, const tVector& pos)
{
	const auto& ball = GetBall(ball_handle);
	double r = gBallRadius;

	tVector ground_pos = pos;
	ground_pos[1] = r + mGround->SampleHeight(ground_pos);
	
	ball->SetPos(ground_pos);
	ball->SetRotation(tQuaternion::Identity());
	ball->SetLinearVelocity(tVector::Zero());
	ball->SetAngularVelocity(tVector::Zero());
}

void cScenarioMultCharRugby::ResetBallTimer()
{
	mRandBallPosTimer = mRand.RandDouble(mRandBallPosTimeMin, mRandBallPosTimeMax);
}

int cScenarioMultCharRugby::GetTargetBallHandle() const
{
	int handle = gInvalidIdx;
	int num_balls = GetNumBalls();
	if (num_balls > 0)
	{
		handle = mBallObjHandles[num_balls - 1];
	}
	return handle;
}


int cScenarioMultCharRugby::FindNearestBall(const tVector& pos) const
{
	int nearest_idx = gInvalidIdx;
	double min_dist = std::numeric_limits<double>::infinity();

	int num_balls = GetNumBalls();
	for (int i = 0; i < num_balls; ++i)
	{
		int curr_handle = mBallObjHandles[i];
		tVector ball_pos = GetBallPos(curr_handle);
		tVector delta = ball_pos - pos;
		delta[1] = 0;
		double dist = delta.squaredNorm();
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_idx = i;
		}
	}

	return nearest_idx;
}
*/
