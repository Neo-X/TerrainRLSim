#pragma once

#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpHike.h"
#include "tinyxml2.h"

class cScenarioMultChar : virtual public cScenarioImitateStepEval, virtual public cScenarioExpHike
{


public:

	struct tAgentData
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tAgentData();
		bool reachedTarget;
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
struct obstacle{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;
};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMultChar();
	virtual ~cScenarioMultChar();

	virtual void Init();
	//void Init(std::shared_ptr<cWorld> world, const tParams& params, const std::string& psteersuite_file);
	virtual const std::shared_ptr<cSimCharacter>& GetSpaceCharacter() const;

	virtual std::string GetName() const;

	virtual double CalcReward();

	virtual double calcRewardForAgent(size_t agent);
	// bool EnableLLCFeedbackReward() const;
	void HandleNewActionUpdate();

	void SyncCharacters();

	int GetCharacterCount();

	// Made public for easier access in rendering, should probably change this eventually
	std::vector<std::shared_ptr<cSimCharacter>> mChars;
	std::vector<tVector> mPrevCOMs;
	std::vector<double> mPrevTimes;

protected:
    //steersuite
	double pathlength;
	std::string steersuite_file;
	std::vector<agent> agents;
	std::vector<obstacle> obstacles;
	std::vector<tVector> pastPositions;
	bool solved;

	double timestamp;
	std::vector<tVector> lastLocations;
	std::vector<double> agentPathLengths;
	std::vector<double> efforts;
	std::vector<int> gettarget;
	
	std::vector<double> rewards;
	std::vector<tVector> previousPositions;
	mutable tVector previousPosition;

	std::vector<double> masses;
	std::vector<tAgentData> agentDatas;
	double mass;

	std::shared_ptr<cSimCharacter> mChar1;

	int mNumChars;

	std::string terrain_file;
	
	double mDensityModulationRange;
	double mInitSpawnRadius;
	double mInitRandTargetBound;
	double mSpawnRadius;
	double mRandTargetBound;
	bool mRewardEffortQuality;
	bool mRandomizeInititalRotation;
	bool mCreateNewGoals;
	bool mUseSimpleReward;
	bool mUseSimpleDistanceReward;
	double mReachTargetBonus;
	bool mUseRepulsiveReward;
	bool mUsePursuitConfig;
	double mTargetRewardWeight;



	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);
	virtual double ComputeTotalCharacterMass(std::shared_ptr<cSimCharacter>& character);

	virtual void SetupGround();

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void BuildChars();
	virtual void BuildCharsSteerSuite();
	virtual void UpdateCharacter(double time_step);
	virtual void Reset();
};
