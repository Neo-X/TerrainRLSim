#pragma once

#include "scenarios/ScenarioImitateTargetEval.h"
#include "scenarios/ScenarioExpImitateStep.h"

class cScenarioImitateStepEval : virtual public cScenarioImitateTargetEval, virtual public cScenarioExpImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateStepEval();
	virtual ~cScenarioImitateStepEval();

	virtual std::string GetName() const;
	virtual bool endOfEpoch() const;


protected:
	std::shared_ptr<cScenarioSimChar> mScene;

	virtual bool HasFallen() const;
};