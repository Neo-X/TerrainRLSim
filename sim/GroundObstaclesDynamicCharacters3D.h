#pragma once

#include "sim/GroundDynamicCharacters3D.h"
#include "sim/SimCharacter.h"
#include "tinyxml2.h"

class cGroundObstaclesDynamicCharacters3D : public cGroundDynamicCharacters3D
{
struct obstacle{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;
};
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundObstaclesDynamicCharacters3D();
	virtual ~cGroundObstaclesDynamicCharacters3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);//,  const std::string& psteersuite_file);
	//steersuite
	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params, const std::string& psteersuite_file);
	virtual void Clear();
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);

	virtual int GetNumObstacles() const;
	virtual const cSimObj& GetObj(int i) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;
	void SampleHeightVel(cSimCharacter* _char, const tVector& pos, double& out_h, tVector& out_vel,
											bool& out_valid_sample) const;

	virtual tVector FindRandFlatPos(const tVector& buffer_size);

	virtual void SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters);
	virtual void SetChar(const std::shared_ptr<cSimCharacter>& character);
	virtual const std::vector<std::shared_ptr<cSimCharacter>> GetChars() const;
	virtual eClass GetGroundClass() const;

	virtual void AddObstacle(std::shared_ptr<cSimObj>& obj);

protected:

	//steersuite
	std::string steersuite_file;
	std::vector<obstacle> obstacles;

	double timestamp;
	std::vector<tVector> lastLocations;
	std::vector<double> agentPathLengths;
	std::vector<double> efforts;
	std::vector<int> gettarget;
	struct tObstacle
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::shared_ptr<cSimObj> mObj;

		tObstacle();
		virtual ~tObstacle();

	};
	struct location{
    double x;
    double y;
    double z;
};
	struct agent {
   		double radius;
   		struct location position;
   		struct location direction;
   		double speed;
   		struct location targetLocation;
   		double desiredSpeed;
   		double timeDuration;
	};

	// This is the vector of 'other characters'
	std::vector<std::shared_ptr<cSimCharacter>> mChars;

	// This is the main character
	std::shared_ptr<cSimCharacter> mChar;
	std::vector<agent> obstacleagents;
	/// List of static obstacles
	std::vector<tObstacle, Eigen::aligned_allocator<tObstacle>> mObstacles;

	virtual int GetBlendParamSize() const;

	virtual void BuildObstacles();

	virtual void ClearObstacles();
	virtual void SortObstacles();
	virtual void BuildObstacle(tObstacle& out_obstacle);
	virtual void BuildObstacle(tObstacle& out_obstacle, int i);


	virtual double FindMaxBoundHeight(const tVector& aabb_min, const tVector& aabb_max) const;
};
