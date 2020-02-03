#pragma once

#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpSoccerGeneral.h"

class cScenarioSoccerGeneralEval : virtual public cScenarioImitateStepEval, virtual public cScenarioExpSoccerGeneral
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioSoccerGeneralEval();
	virtual ~cScenarioSoccerGeneralEval();

	virtual std::string GetName() const;

	protected:
};
