#include "sim/GroundObstaclesDynamicCharacters3D.h"
#include "sim/TerrainGen3D.h"
#include "sim/SimBox.h"
#include <iostream>
#include "tinyxml2.h"
#include <fstream>
#include <math.h>
using namespace tinyxml2;

const double gDefaultHeight = 0;
const tVector gObstaclePosMin = tVector(-2, 0, -2, 0);
const tVector gObstaclePosMax = tVector(2, 0, 2, 0);
const double E_S = 2.23;
const double E_W = 1.26;
const double MASS = 1.0;
const double targetrange = 1.0;


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
	steersuite_file = "";
	cGroundDynamicCharacters3D::Init(world, params);
	BuildObstacles();

	std::ofstream myFile("data.csv");
	myFile << "" << std::endl;
	myFile.close();
	timestamp = 0;
}

//steersuite
void cGroundObstaclesDynamicCharacters3D::Init(std::shared_ptr<cWorld> world, 
							const tParams& params, const std::string& psteersuite_file)
{
	steersuite_file = psteersuite_file;
	if(steersuite_file != ""){
		printf("\n\nGroundObstaclesDynamicCharacters3D.cpp steersuite_file: %s\n\n\n", steersuite_file.c_str());
	XMLDocument doc;

	char path[] = "/home/gberseth/playground/TerrainRLSim/";
	char* p = path;
	printf("type of steersuite_file.c_str(): %s\n", typeid(steersuite_file.c_str()).name());

	char *filename = new char[strlen(p)+strlen(steersuite_file.c_str())+1];
	strcpy(filename,p);
	strcat(filename,steersuite_file.c_str());

    doc.LoadFile(filename);

    XMLElement * pRoot = doc.FirstChildElement();

    XMLElement *first = pRoot->FirstChildElement();


	for (XMLElement* child = first; child != NULL; child = child->NextSiblingElement())
    {
        if(strncmp(child->Name(), "agent", 5) == 0){

            struct agent a = {};
            XMLElement *condition = child->FirstChildElement("initialConditions");

            condition->FirstChildElement("radius")->QueryDoubleText(&a.radius);
            XMLElement *position = condition->FirstChildElement("position");

            position->FirstChildElement("x")->QueryDoubleText(&a.position.x);
            position->FirstChildElement("y")->QueryDoubleText(&a.position.y);
            position->FirstChildElement("z")->QueryDoubleText(&a.position.z);

            XMLElement *direction = condition->FirstChildElement("direction");

            direction->FirstChildElement("x")->QueryDoubleText(&a.direction.x);
            direction->FirstChildElement("y")->QueryDoubleText(&a.direction.y);
            direction->FirstChildElement("z")->QueryDoubleText(&a.direction.z);

            condition->FirstChildElement("speed")->QueryDoubleText(&a.speed);

            XMLElement *statictarget = child->FirstChildElement("goalSequence")->FirstChildElement("seekStaticTarget");

            XMLElement *targetLocation = statictarget->FirstChildElement("targetLocation");
            targetLocation->FirstChildElement("x")->QueryDoubleText(&a.targetLocation.x);
            targetLocation->FirstChildElement("y")->QueryDoubleText(&a.targetLocation.y);
            targetLocation->FirstChildElement("z")->QueryDoubleText(&a.targetLocation.z);

            statictarget->FirstChildElement("desiredSpeed")->QueryDoubleText(&a.desiredSpeed);
            statictarget->FirstChildElement("timeDuration")->QueryDoubleText(&a.timeDuration);

			obstacleagents.push_back(a);

        }
        if(strncmp(child->Name(), "obstacle", 8) == 0){

            struct obstacle o = {};
            child->FirstChildElement("xmin")->QueryDoubleText(&o.xmin);
            child->FirstChildElement("xmax")->QueryDoubleText(&o.xmax);
            child->FirstChildElement("ymin")->QueryDoubleText(&o.ymin);
            child->FirstChildElement("ymax")->QueryDoubleText(&o.ymax);
            child->FirstChildElement("zmin")->QueryDoubleText(&o.zmin);
            child->FirstChildElement("zmax")->QueryDoubleText(&o.zmax);

			//printf("\nGroundObstaclesDynamicCharacters3D.cpp xmin: %f\n", o.xmin);
			obstacles.push_back(o);
        }
	}
	
    }
	cGroundDynamicCharacters3D::Init(world, params);
	BuildObstacles();

	std::ofstream myFile("data.csv");
	myFile << "" << std::endl;
	myFile.close();
	timestamp = 0;
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

const std::vector<std::shared_ptr<cSimCharacter>> cGroundObstaclesDynamicCharacters3D::GetChars() const
{
	std::vector<std::shared_ptr<cSimCharacter>> out;

	out.push_back(mChar);
	for (size_t a=0; a < mChars.size(); a++)
	{
		out.push_back(mChars[a]);
	}
	return out;
}

double cGroundObstaclesDynamicCharacters3D::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	//printf("\n\nGroundObstaclesDynamicCharacters3D.cpp SampleHeight \n\n");
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
	//printf("\n\nGroundObstaclesDynamicCharacters3D.cpp SampleHeightVel \n\n");
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
	//printf("\n\nGroundObstaclesDynamicCharacters3D.cpp SampleHeightVel SimCharacter\n\n");
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
	if(steersuite_file != ""){
		num_obstacles = obstacles.size();
	}

	mObstacles.clear();
	mObstacles.reserve(num_obstacles);

	for (int i = 0; i < num_obstacles; ++i)
	{
		tObstacle curr_obstacle;
		if(steersuite_file != ""){
			printf("\n\nGroundObstaclesDynamicCharacters3D.cpp  steersuite BuildObstacles\n\n");
			BuildObstacle(curr_obstacle, i);
		}
		else{
			printf("\n\nGroundObstaclesDynamicCharacters3D.cpp not steersuite BuildObstacles\n\n");
			BuildObstacle(curr_obstacle);
		}
		
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
	
	/*
	//printf("\nGroundObstaclesDynamicCharacters3D.cpp updata here\n");
	std::ofstream myFile("data.csv", std::ios::app);
	if(lastLocations.empty())
	{
		
		tVector rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);
		double angle = acos(obstacleagents[0].direction.x/
							(sqrt(obstacleagents[0].direction.x*obstacleagents[0].direction.x+
									obstacleagents[0].direction.y*obstacleagents[0].direction.y+
										obstacleagents[0].direction.z*obstacleagents[0].direction.z)));
		tQuaternion new_rotation2 = cMathUtil::AxisAngleToQuaternion(rotation_axis, angle);
		mChar->SetRootTransform(tVector(obstacleagents[0].position.x, 0.82,
										obstacleagents[0].position.z, 0.0), new_rotation2);

		//tvector targetLocation
		mChar->SetCurrentGroundTarget(tVector(obstacleagents[0].targetLocation.x, obstacleagents[0].targetLocation.y,
											obstacleagents[0].targetLocation.z, 0.0));
											
	myFile << "timestamp,";
	myFile << "# collisions,";
	myFile << "agent 0 path length,";
	myFile << "agent 0 effort,";
	myFile << "agent 0 get target,";

	tVector newPosition = mChar->GetRootPos();
	lastLocations.push_back(newPosition);
	agentPathLengths.push_back(0.0);
	efforts.push_back(0.0);
	gettarget.push_back(0);
	
	int i = 1;
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it) 
	{
		
		tVector rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);
		double angle = acos(obstacleagents[i].direction.x/
							(sqrt(obstacleagents[i].direction.x*obstacleagents[i].direction.x+
									obstacleagents[i].direction.y*obstacleagents[i].direction.y+
										obstacleagents[i].direction.z*obstacleagents[i].direction.z)));
		tQuaternion new_rotation2 = cMathUtil::AxisAngleToQuaternion(rotation_axis, angle);
		(*it)->SetRootTransform(tVector(obstacleagents[i].position.x, 0.82,
										obstacleagents[i].position.z, 0.0), new_rotation2);

		//tvector targetLocation
		(*it)->SetCurrentGroundTarget(tVector(obstacleagents[i].targetLocation.x, obstacleagents[i].targetLocation.y,
											obstacleagents[i].targetLocation.z, 0.0));

		
		tVector newPosition = (*it)->GetRootPos();
		lastLocations.push_back(newPosition);
		agentPathLengths.push_back(0.0);
		efforts.push_back(0.0);
		gettarget.push_back(0);
		myFile << "agent " << std::to_string(i) << " path length,";
		myFile << "agent " << std::to_string(i) << " effort,";
		myFile << "agent " << std::to_string(i) << "get target,";
		i++;
	}
	myFile << std::endl;
	timestamp = 0.0;
	}
	//printf("\n\nGroundObstaclesDynamicCharacters3D.cpp steersuite update here\n\n");
	
	
	//path length
	tVector newPosition = mChar->GetRootPos();
	agentPathLengths[0] += sqrt((newPosition[0]-lastLocations[0][0])*(newPosition[0]-lastLocations[0][0])
						+(newPosition[2]-lastLocations[0][2])*(newPosition[2]-lastLocations[0][2]));
	lastLocations[0] = newPosition;

	

	//effort
	tVector velocity = mChar->CalcCOMVel();
	double velocitysquared = velocity[0]*velocity[0]+velocity[1]*velocity[1]+velocity[2]*velocity[2];
	double effort = MASS * (E_S + E_W * velocitysquared)*time_elapsed;
	efforts[0] += effort;
	//printf("COM velocity is: %f, %f, %f\n", velocity[0], velocity[1], velocity[2]);
	//printf("effort now is: %f\n", effort);

	//get target
	
	//printf("current ground target: %f %f %f\n", currentgroundtarget[0], currentgroundtarget[1], currentgroundtarget[2]);
	//printf("target location: %f %f %f\n", targetLocation[0], targetLocation[1], targetLocation[2]);


	int i = 1;
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it) 
	{	
		//path length
		tVector newPosition = (*it)->GetRootPos();
		
		agentPathLengths[i] += sqrt((newPosition[0]-lastLocations[i][0])*(newPosition[0]-lastLocations[i][0])
						+(newPosition[2]-lastLocations[i][2])*(newPosition[2]-lastLocations[i][2]));
		lastLocations[i] = newPosition;

		//effort
		tVector velocity = (*it)->CalcCOMVel();
		double velocitysquared = velocity[0]*velocity[0]+velocity[1]*velocity[1]+velocity[2]*velocity[2];
		double effort = MASS * (E_S + E_W * velocitysquared)*time_elapsed;
		efforts[i] += effort;

		//get target

		
		//printf("current ground target: %f %f %f\n", currentgroundtarget[0], currentgroundtarget[1], currentgroundtarget[2]);
		//printf("target location: %f %f %f\n", targetLocation[0], targetLocation[1], targetLocation[2]);

		i++;
	}
	
	myFile << timestamp << ",";
	myFile << ",";
	for(int j = 0; j < agentPathLengths.size(); j++){
		myFile << agentPathLengths[j] << ";";
		myFile << efforts[j] << ",";
		if(gettarget[j] == 0){
			myFile << "false,";
		}
		else{
			myFile << "true,";
		}
		
	}
	myFile << std::endl;
	
	timestamp += time_elapsed;
	*/
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
	//printf("\n\n\nGroundObstaclesDynamicCharacters3D: %s\n\n\n", mArgs[0]);
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
void cGroundObstaclesDynamicCharacters3D::BuildObstacle(tObstacle& out_obstacle, int i)
{
	//printf("\n\n\nGroundObstaclesDynamicCharacters3D: %s\n\n\n", mArgs[0]);
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

	
	//steersuite
	if(steersuite_file != ""){
		size = tVector(obstacles[i].xmax-obstacles[i].xmin,
										obstacles[i].ymax-obstacles[i].ymin,
										obstacles[i].zmax-obstacles[i].zmin, 0.0);
	 	pos_start = tVector((obstacles[i].xmin+obstacles[i].xmax)/2.0,
											(obstacles[i].ymin+obstacles[i].ymax)/2.0,
											(obstacles[i].zmin+obstacles[i].zmax)/2.0, 0.0);
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
