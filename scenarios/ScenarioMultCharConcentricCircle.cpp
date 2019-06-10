#include "ScenarioMultCharConcentricCircle.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/WaypointController.h"

cScenarioMultCharConcentricCircle::cScenarioMultCharConcentricCircle() :
	cScenarioMultChar()
{
	EnableTargetPos(true);
	EnableRandTargetPos(true);
	mNumChars=0;
	mSpawnRadius=10.0;
	mRandTargetBound = 10.0;
}

cScenarioMultCharConcentricCircle::~cScenarioMultCharConcentricCircle()
{
}

void cScenarioMultCharConcentricCircle::Init()
{
	cScenarioMultChar::Init();
}

void cScenarioMultCharConcentricCircle::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioMultChar::ParseMiscArgs(parser);  // BRANDON WORKING HERE ON PARSIN GINPUTS
	
	parser->ParseInt("num_characters", mNumChars);
	parser->ParseDouble("spawn_radius", mSpawnRadius);
	parser->ParseDouble("rand_target_bound", mRandTargetBound);
}


std::string cScenarioMultCharConcentricCircle::GetName() const
{
	return "Scenario similar to Hike but with multiple characters";
}

void cScenarioMultCharConcentricCircle::BuildChars()
{
	// Initialize all of the characters
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

		GenerateInitialTransform(newChar, i+1, rand_rot);
		newChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(newChar));

		previousPositions.push_back(newChar->GetRootPos());
		masses.push_back(ComputeTotalCharacterMass(newChar));
		mChars.push_back(newChar);
		rewards.push_back(0.0);
	}	


	//////////////////////////////////////////////////////////////
	GenerateInitialTransform(mChar, 0, rand_rot);
	previousPosition = mChar->GetRootPos();
	mass = ComputeTotalCharacterMass(mChar);
	mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));
}


void cScenarioMultCharConcentricCircle::GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character,
																size_t agentNum, double rand_rot)
{
	// Find a clear ground plane point to place the character
	// do
	tVector rotation_axis;
	double rotation_angle;
	double circle_rad = mSpawnRadius + mRand.RandDouble(-2, 1);
	tVector origin = tVector(0,0,0,0);
	tVector dist = tVector(1.0,0,0,0);
	rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);
	double portion = (M_PI * 2) / (mNumChars + 1);
	/// To add more noise to the initial circle positions
	double rand_circle_rot = rand_rot;
	{
		tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, ((portion * agentNum) + mRand.RandDouble(-portion/3.0, portion/3.0)) + rand_circle_rot);
		tVector new_pos = cMathUtil::QuatRotVec(new_rotation, dist * circle_rad);
		new_pos[1] = 0.82;
		character->Reset();

		// Handle position
		// tVector root_pos1 = character->GetRootPos();
		// root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0.82, mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);
		
		// bool freeSpace = true;

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
		
		// character->SetRootPos(root_pos1);
		// character->SetRootPos0(root_pos1);

		// ResolveCharGroundIntersect(character);

		// Handle rotation

		// rotation_angle = mRand.RandDouble(-M_PI, M_PI);
		// Look the opposite direction
		// tQuaternion new_rotation2 = cMathUtil::AxisAngleToQuaternion(rotation_axis, (((portion * agentNum) + mRand.RandDouble(-portion/3.0, portion/3.0)) - M_PI) + (rand_circle_rot + mRand.RandDoubleNorm(0, 0.025) ));
		tQuaternion new_rotation2 = cMathUtil::AxisAngleToQuaternion(rotation_axis, (((portion * agentNum)) - M_PI) + (rand_circle_rot + mRand.RandDoubleNorm(0, 0.025) ));
		
		// TODO: Need to fix this, character initial link velocities need to be lined up with this rotation
		// character->SetRootRotation(new_rotation);
		// std::cout << "new_pos: " << new_pos <<std::endl;
		character->SetRootTransform(new_pos, new_rotation2);
		new_pos[1] = 0.0;
		character->SetCurrentGroundTarget(-new_pos);

	}
	// while (character->GetRootPos()[1] - 0 > 0.82); //  Ensures character finds a spot on the flat ground
}

void cScenarioMultCharConcentricCircle::Reset()
{
	cScenarioExpHike::Reset();

	size_t agentNum = 0;
	double rand_rot = mRand.RandDouble(-M_PI, M_PI);
	GenerateInitialTransform(mChar, agentNum, rand_rot);
	// previousPosition = mChar->GetRootPos();
	// mChar->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D(mChar));


	// Find random positions somewhere on the open ground for the rest of the characters
	std::vector<std::shared_ptr<cSimCharacter>>::iterator it;
	std::vector<tVector>::iterator pos;

	for(it = mChars.begin(), pos = previousPositions.begin(); 
		it != mChars.end() && pos != previousPositions.end(); 
		++it, ++pos) 
	{
		agentNum++;
		GenerateInitialTransform((*it), agentNum, rand_rot);
		(*pos) = (*it)->GetRootPos();
		// (*it)->SetCurrentGroundTarget(CalcTargetPosObstaclesDynamicCharacters3D((*it)));
		// (*it)->SetRootRotation(mChar->GetRootRotation());
	}

	for (size_t a =0; a < mChars.size(); a++)
	{
		mPrevCOMs[a] = mChars[a]->CalcCOM();
	}
}

tVector cScenarioMultCharConcentricCircle::CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character)
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

