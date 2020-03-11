#pragma once

// hack hack hack
// get rid of this ASAP
//#define HACK_SOCCER_LLC

#include "scenarios/ScenarioExpImitateStep.h"

class cScenarioExpHikeGeneral : virtual public cScenarioExpImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	cScenarioExpHikeGeneral();
	virtual ~cScenarioExpHikeGeneral();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void SetBufferSize(int size);
	virtual void EnableExplore(bool enable);

	virtual std::string GetName() const;

	virtual double CalcReward() const;

protected:

	double mTargetSpeed;
	double mReachTargetBonus;

	virtual void ResetParams();
	virtual void ResetKinChar();

	virtual void PreSubstepUpdate(double time_step);
	virtual void PostSubstepUpdate(double time_step);
	virtual void HandleNewActionUpdate();
	virtual void HandleFallUpdate();

	virtual tVector CalcTargetPosDefault();
	virtual int GetTargetPosTrail3dForwardSegs() const;
};
