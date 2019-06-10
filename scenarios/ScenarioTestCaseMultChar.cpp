#include "ScenarioTestCaseMultChar.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/WaypointController.h"

cScenarioTestCaseMultChar::cScenarioTestCaseMultChar() :
cScenarioSpaceMultChar()
{
}

cScenarioTestCaseMultChar::~cScenarioTestCaseMultChar()
{
}

void cScenarioTestCaseMultChar::Init()
{
	cScenarioSpaceMultChar::Init();
}

void cScenarioTestCaseMultChar::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioSpaceMultChar::ParseMiscArgs(parser);
	
	// parser->ParseInt("num_characters", mNumChars);
	// parser->ParseDouble("spawn_radius", mSpawnRadius);
}

std::string cScenarioTestCaseMultChar::GetName() const
{
	return "Scenario scene where the layout is generated from parameters";
}

void cScenarioTestCaseMultChar::GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character,
													size_t agentNum, double rand_rot)
{
	// Find a clear ground plane point to place the character
	character->Reset();

	bool freeSpace = true;
	tVector unit_vec = tVector(1,1,1,0);
	// Find any intersections with previously placed characters
	// Note we want to avoid any intersections with any characters, thus the inner loop which checks all characters
	// given the current generated starting position, and generates a new one if any intersection is found
	tVector root_pos1 = character->GetRootPos();
	do
	{
		freeSpace = true;
		// Handle position
		root_pos1 = tVector(mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0.0, mRand.RandDouble(-mSpawnRadius, mSpawnRadius), 0);

		tVector char_aabb_min;
		tVector char_aabb_max;

		// character->CalcAABB(char_aabb_min, char_aabb_max);
		// char_aabb_min = char_aabb_min - unit_vec;
		// char_aabb_max = char_aabb_max + unit_vec;

		// tVector other_char_aabb_min;
		// tVector other_char_aabb_max;

		for(std::vector<std::shared_ptr<cSimCharacter>>::iterator it = mChars.begin(); it != mChars.end(); ++it)
		{
			// character->CalcAABB(other_char_aabb_min, other_char_aabb_max);
			// and is within our bounds then this is not a free space, also check default character here
			if ( (root_pos1 - (*it)->GetRootPos()).norm() < 1.0 )
			// if (cMathUtil::ContainsAABB(char_aabb_min, char_aabb_max, other_char_aabb_min, other_char_aabb_max))
			{
				// std::cout << "character character intersection:" << std::endl;
				freeSpace = false;
				break;
			}
		}
		// mChar->CalcAABB(other_char_aabb_min, other_char_aabb_max);
		// and is within our bounds then this is not a free space, also check default character here
		if ( (root_pos1 - mChar->GetRootPos()).norm() < 1.0)
		// if (cMathUtil::ContainsAABB(char_aabb_min, char_aabb_max, other_char_aabb_min, other_char_aabb_max))
		{
			// std::cout << "character character2 intersection:" << std::endl;
			freeSpace = false;
			break;
		}

		// std::cout << "char_aabb_min: " << char_aabb_min << " char_aabb_max: " << char_aabb_max << std::endl;
		for (int i = 0; i < mGround->GetNumObstacles(); ++i)
		{
			const auto& obj = mGround->GetObj(i);
			tVector aabb_min;
			tVector aabb_max;
			obj.CalcAABB(aabb_min, aabb_max);
			aabb_min = aabb_min - unit_vec;
			aabb_max = aabb_max + unit_vec;
			// std::cout << "testing character obstacle intersection:" << std::endl;
			// if (cMathUtil::ContainsAABB(char_aabb_min, char_aabb_max, aabb_min, aabb_max))
			if (cMathUtil::ContainsAABB(root_pos1, aabb_min, aabb_max))
			{
				// std::cout << "character obstacle intersection:" << std::endl;
				freeSpace = false;
				break;
			}
		}


	} while (!freeSpace);

	// character->SetRootPos(root_pos1);
	// character->SetRootPos0(root_pos1);

	// ResolveCharGroundIntersect(character);
	root_pos1[1] = 0.82;

	// Handle rotation
	tVector rotation_axis;
	double rotation_angle;

	rotation_angle = 0;
	rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);

	tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, rotation_angle);
	// std::cout << "setting scenario space char" << std::endl;
	// character->SetRootRotation(new_rotation);
	character->SetRootTransform(root_pos1, new_rotation);


}

tVector cScenarioTestCaseMultChar::CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character)
{
	tVector root_pos1 = tVector::Zero();
	bool freeSpace = true;
	tVector unit_vec = tVector(12,0,0,0);
	root_pos1 = character->GetRootPos();
	// Find any intersections with previously placed characters
	// Note we want to avoid any intersections with any characters, thus the inner loop which checks all characters
	// given the current generated starting position, and generates a new one if any intersection is found
	// tVector root_pos1 = character->GetRootPos();
	// std::cout << "testing target obstacle intersection:" << std::endl;


	root_pos1[1] = 0.0;
	root_pos1 = root_pos1 + unit_vec;
	return root_pos1;
}


void cScenarioTestCaseMultChar::Reset()
{
	cScenarioSpaceMultChar::Reset();

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

