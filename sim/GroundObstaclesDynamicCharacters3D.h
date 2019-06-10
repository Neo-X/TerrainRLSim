#pragma once

#include "sim/GroundDynamicCharacters3D.h"
#include "sim/SimCharacter.h"

class cGroundObstaclesDynamicCharacters3D : public cGroundDynamicCharacters3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundObstaclesDynamicCharacters3D();
	virtual ~cGroundObstaclesDynamicCharacters3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
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
	virtual eClass GetGroundClass() const;

protected:

	struct tObstacle
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::shared_ptr<cSimObj> mObj;

		tObstacle();
		virtual ~tObstacle();

	};

	// This is the vector of 'other characters'
	std::vector<std::shared_ptr<cSimCharacter>> mChars;

	// This is the main character
	std::shared_ptr<cSimCharacter> mChar;

	/// List of static obstacles
	std::vector<tObstacle, Eigen::aligned_allocator<tObstacle>> mObstacles;

	virtual int GetBlendParamSize() const;

	virtual void BuildObstacles();
	virtual void ClearObstacles();
	virtual void SortObstacles();
	virtual void BuildObstacle(tObstacle& out_obstacle);

	virtual double FindMaxBoundHeight(const tVector& aabb_min, const tVector& aabb_max) const;
};
