#pragma once

#include "scenarios/ScenarioMultChar.h"

class cScenarioMultCharRugby : virtual public cScenarioMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMultCharRugby();
	virtual ~cScenarioMultCharRugby();

	virtual void Init();

	virtual std::string GetName() const;

	virtual void SetBallPos(const tVector& pos);
	// virtual int GetNumBalls() const;
	// virtual void RemoveObj(int handle);
	virtual const std::shared_ptr<cSimObj>& GetBall() const;

	virtual double CalcReward();
	virtual double calcRewardForAgent(size_t agent);

	virtual bool endOfEpoch() const;

protected:
	int ballObjHandle;
	std::shared_ptr<cSimObj> ball;
	// bool mRemoveBallAtGoal;

	// tVector mPrevBallPos;

	// double mRandBallPosTimeMin;
	// double mRandBallPosTimeMax;
	// double mRandBallPosTimer;
	
	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t agentNum);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);

	//virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void BuildChars();
	virtual void Reset();

	virtual void ResetParams();
	virtual bool CheckResetTarget() const;
	virtual void ClearObjs();

	// virtual double GetRandTargetMaxDist() const;
	// virtual double GetRandBallMaxDist() const;
	virtual void HandleNewActionUpdate();

	// virtual bool EndEpisode() const;

	// virtual void BuildBalls();
	virtual int BuildBall();
	// virtual const std::shared_ptr<cSimObj>& GetBall(int ball_handle) const;
	virtual tVector GetBallPos() const;
	// virtual tVector GetBallPos(int ball_handle) const;

	// virtual tVector CalcTargetPosDefault();
	// virtual void UpdateBallPos(double time_elapsed);
	// virtual void ResetBallPosAll();
	// virtual void ResetBallPos(int ball_handle);
	// virtual void SetBallPos(int ball_handle, const tVector& pos);
	// virtual void SetBallPos(const tVector& pos);
	// virtual void ResetBallTimer();

	// virtual int GetTargetBallHandle() const;
	// virtual void UpdateTargetBall();
	// virtual int FindNearestBall(const tVector& pos) const;

	virtual void UpdateCharacter(double time_step);
};
