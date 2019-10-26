#pragma once

#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpHike.h"

class cScenarioMultChar : virtual public cScenarioImitateStepEval, virtual public cScenarioExpHike
{
public:

	struct tAgentData
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tAgentData();
		bool reachedTarget;
	};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMultChar();
	virtual ~cScenarioMultChar();

	virtual void Init();
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
	std::vector<double> rewards;
	
	std::vector<tVector> previousPositions;
	mutable tVector previousPosition;

	std::vector<double> masses;
	std::vector<tAgentData> agentDatas;
	double mass;

	std::shared_ptr<cSimCharacter> mChar1;

	int mNumChars;

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
	virtual void UpdateCharacter(double time_step);
	virtual void Reset();
};
