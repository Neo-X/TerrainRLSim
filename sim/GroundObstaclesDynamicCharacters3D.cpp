#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/TerrainGen3D.h"
#include "sim/SimBox.h"
#include <iostream>

const double gDefaultHeight = 0;
const tVector gObstaclePosMin = tVector(-2, 0, -2, 0);
const tVector gObstaclePosMax = tVector(2, 0, 2, 0);

cGroundObstaclesDynamicCharacters3D::tObstacle::tObstacle()
{
}

cGroundObstaclesDynamicCharacters3D::tObstacle::~tObstacle()
{
}

cGroundObstaclesDynamicCharacters3D::cGroundObstaclesDynamicCharacters3D() :
		cGroundDynamicCharacters3D()
{
}

cGroundObstaclesDynamicCharacters3D::~cGroundObstaclesDynamicCharacters3D()
{
}

cGroundObstaclesDynamicCharacters3D::eClass cGroundObstaclesDynamicCharacters3D::GetGroundClass() const
{
	return eClassObstaclesDynamicCharacters3D;
}

void cGroundObstaclesDynamicCharacters3D::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	cGroundDynamicCharacters3D::Init(world, params);
	BuildObstacles();
}

void cGroundObstaclesDynamicCharacters3D::Clear()
{
	cGroundDynamicCharacters3D::Clear();
	ClearObstacles();
}

int cGroundObstaclesDynamicCharacters3D::GetBlendParamSize() const
{
	return static_cast<int>(cTerrainGen3D::eParamsMax);
}

int cGroundObstaclesDynamicCharacters3D::GetNumObstacles() const
{
	return static_cast<int>(mObstacles.size());
}

const cSimObj& cGroundObstaclesDynamicCharacters3D::GetObj(int i) const
{
	return *mObstacles[i].mObj;
}


void cGroundObstaclesDynamicCharacters3D::SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters)
{
	mChars = characters;
}

void cGroundObstaclesDynamicCharacters3D::SetChar(const std::shared_ptr<cSimCharacter>& character)
{
	mChar = character;
}

double cGroundObstaclesDynamicCharacters3D::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	double h;
	tVector vel;
	SampleHeightVel(pos, h, vel, out_valid_sample);
	/// Hack because character are sampling themselves....
	h=0;
	return h;
}

void cGroundObstaclesDynamicCharacters3D::SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel,
													bool& out_valid_sample) const
{
	out_h = gDefaultHeight;
	out_vel = tVector::Zero();

	bool foundIntersectedChar = false;

	// Check to see if this sample point contains any of the other characters
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it) 
	{
		tVector aabb_min;
		tVector aabb_max;
		(*it)->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = (*it)->CalcCOMVel(); // Works for new LLC...
			// out_vel = (*it)->GetRootVel(); // Works for old LLC
			foundIntersectedChar = true;
			break;
		}
	}

	// If we havent found one intersection amongst the additional character, then check the default one as well
	if (!foundIntersectedChar)
	{
		tVector aabb_min;
		tVector aabb_max;
		mChar->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = mChar->CalcCOMVel();
			// out_vel = mChar->GetRootVel(); // Works for old LLC
			foundIntersectedChar = true;
		}
	}

	for (int i = 0; i < GetNumObstacles() && (!foundIntersectedChar); ++i)
	{
		const tObstacle& obstacle = mObstacles[i];
		const auto& obj = obstacle.mObj;
		tVector aabb_min;
		tVector aabb_max;
		obj->CalcAABB(aabb_min, aabb_max);

		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = obj->GetLinearVelocity();
			break;
		}
	}

	// out_h = SampleHeight(pos, out_valid_sample);
	out_valid_sample = true;
}

void cGroundObstaclesDynamicCharacters3D::SampleHeightVel(cSimCharacter* _char, const tVector& pos, double& out_h, tVector& out_vel,
										bool& out_valid_sample) const
{
	out_h = gDefaultHeight;
	out_vel = tVector::Zero();

	bool foundIntersectedChar = false;

	// Check to see if this sample point contains any of the other characters
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it)
	{
		tVector aabb_min;
		tVector aabb_max;
		(*it)->CalcAABB(aabb_min, aabb_max);

		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = (*it)->CalcCOMVel() - _char->CalcCOMVel(); // Works for new LLC...
			// out_vel = (*it)->GetRootVel(); // Works for old LLC
			foundIntersectedChar = true;
			break;
		}
	}

	// If we havent found one intersection amongst the additional character, then check the default one as well
	if (!foundIntersectedChar)
	{
		tVector aabb_min;
		tVector aabb_max;
		mChar->CalcAABB(aabb_min, aabb_max);

		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = mChar->CalcCOMVel() - _char->CalcCOMVel();
			// out_vel = mChar->GetRootVel(); // Works for old LLC
			foundIntersectedChar = true;
		}
	}

	for (int i = 0; i < GetNumObstacles() && (!foundIntersectedChar); ++i)
	{
		const tObstacle& obstacle = mObstacles[i];
		const auto& obj = obstacle.mObj;
		tVector aabb_min;
		tVector aabb_max;
		obj->CalcAABB(aabb_min, aabb_max);

		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = obj->GetLinearVelocity() - _char->CalcCOMVel();
			break;
		}
	}

	// out_h = SampleHeight(pos, out_valid_sample);
	out_valid_sample = true;
}

tVector cGroundObstaclesDynamicCharacters3D::FindRandFlatPos(const tVector& buffer_size)
{
	const double valid_h = gDefaultHeight;
	const tVector& bound_min = gObstaclePosMin;
	const tVector& bound_max = gObstaclePosMax;

	tVector pos = tVector::Zero();
	do
	{
		double u = mRand.RandDouble(0.2, 0.8);
		double v = mRand.RandDouble(0.2, 0.8);
		pos[0] = (1 - u) * bound_min[0] + u * bound_max[0];
		pos[2] = (1 - v) * bound_min[2] + v * bound_max[2];
		pos[1] = FindMaxBoundHeight(pos - 0.5 * buffer_size, pos + 0.5 * buffer_size);
	} while (pos[1] != valid_h);

	return pos;
}

void cGroundObstaclesDynamicCharacters3D::BuildObstacles()
{
	int num_obstacles = static_cast<int>(mBlendParams[cTerrainGen3D::eParamsNumObstacles]);
	// int num_obstacles = 2;
	mObstacles.clear();
	mObstacles.reserve(num_obstacles);

	for (int i = 0; i < num_obstacles; ++i)
	{
		tObstacle curr_obstacle;
		BuildObstacle(curr_obstacle);
		mObstacles.push_back(curr_obstacle);
	}

	SortObstacles();
}

void cGroundObstaclesDynamicCharacters3D::ClearObstacles()
{
	mObstacles.clear();
}

void cGroundObstaclesDynamicCharacters3D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	cGroundDynamicCharacters3D::Update(time_elapsed, bound_min, bound_max);

	if (GetNumObstacles() == 0)
	{
		BuildObstacles();
	}

}

void cGroundObstaclesDynamicCharacters3D::SortObstacles()
{
	std::sort(mObstacles.begin(), mObstacles.end(),
		[](const tObstacle& a, const tObstacle& b)
	{
		tVector aabb_min0;
		tVector aabb_max0;
		tVector aabb_min1;
		tVector aabb_max1;
		a.mObj->CalcAABB(aabb_min0, aabb_max0);
		b.mObj->CalcAABB(aabb_min1, aabb_max1);

		return aabb_max0[1] > aabb_max1[1];
	});
}

void cGroundObstaclesDynamicCharacters3D::BuildObstacle(tObstacle& out_obstacle)
{
	/*
	const double min_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth0];
	const double max_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth1];
	const double min_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight0];
	const double max_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight1];
	const double min_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed0];
	const double max_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed1];
	const double speed_lerp_pow = mBlendParams[cTerrainGen3D::eParamsObstacleSpeedLerpPow];
	*/
	/// Hack to create a set of box obstacles from a list.
	/// {x_pos, y_pos, x_width, y_width, ....}
	std::vector<double> obs_params = {-5,5,19,1,-5,-5,19,1, 4,3.5,1,4,4,-3.5,1,4};
	int num_obstacles = static_cast<int>(mBlendParams[cTerrainGen3D::eParamsNumObstacles]);
	const double min_w = 1;
	const double max_w = 2;
	const double min_h = 1;
	const double max_h = 2;

	tVector size = tVector(mRand.RandDouble(min_w, max_w),
							mRand.RandDouble(min_h, max_h),
							mRand.RandDouble(min_w, max_w), 0);
	tVector pos_start = tVector(mRand.RandDouble(mBlendParams[cTerrainGen3D::eParamsObstaclePositionMin],
												mBlendParams[cTerrainGen3D::eParamsObstaclePositionMax]), 0,
								mRand.RandDouble(mBlendParams[cTerrainGen3D::eParamsObstaclePositionMin],
										mBlendParams[cTerrainGen3D::eParamsObstaclePositionMax]), 0);
	if (num_obstacles == 4 )
	{
		size = tVector(obs_params[(4* mObstacles.size()) + 2],
								3.0,
								obs_params[(4* mObstacles.size()) +3], 0);
		pos_start = tVector(obs_params[(4* mObstacles.size()) +0], 0,
				obs_params[(4* mObstacles.size()) +1], 0);
		// std::cout << "num obs: " << mObstacles.size() << std::endl;
	}

	cSimBox::tParams params;
	params.mSize = size;
	params.mPos = pos_start;
	params.mFriction = mParams.mFriction;
	/// make obstacles static
	params.mMass = 0.0;

	std::shared_ptr<cSimBox> box = std::shared_ptr<cSimBox>(new cSimBox());
	box->Init(mWorld, params);
	box->UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
	box->SetKinematicObject(false);

	// out_obstacle.mPosStart = pos_start;
	// out_obstacle.mPosEnd = pos_end;
	// out_obstacle.mSpeed = speed;
	// out_obstacle.mPhase = mRand.RandDouble(0, 1);
	// out_obstacle.mDir = mRand.FlipCoin() ? tObstacle::eDirForward : tObstacle::eDirBackward;
	out_obstacle.mObj = box;

	// out_obstacle->SetPos(pos_start);
	// out_obstacle = box;
}

void cGroundObstaclesDynamicCharacters3D::AddObstacle(std::shared_ptr<cSimObj>& obj)
{
	tObstacle curr_obstacle;
	// BuildObstacle(curr_obstacle);
	curr_obstacle.mObj = obj;
	mObstacles.push_back(curr_obstacle);
}



double cGroundObstaclesDynamicCharacters3D::FindMaxBoundHeight(const tVector& aabb_min, const tVector& aabb_max) const
{
	double h = gDefaultHeight;

	for (int i = 0; i < GetNumObstacles(); ++i)
	{
		const tObstacle& obstacle = mObstacles[i];
		const auto& obj = obstacle.mObj;
		tVector obj_aabb_min;
		tVector obj_aabb_max;
		obj->CalcAABB(obj_aabb_min, obj_aabb_max);

		if (cMathUtil::IntersectAABBXZ(obj_aabb_min, obj_aabb_max, aabb_min, aabb_max))
		{
			h = obj_aabb_max[1];
			break;
		}
	}

	return h;
}
