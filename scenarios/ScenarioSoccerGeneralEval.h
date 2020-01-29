#pragma once

#include "scenarios/ScenarioExpImitateStep.h"
#include "scenarios/ScenarioExpSoccerGeneral.h"

class cScenarioSoccerGeneralEval : virtual public cScenarioExpImitateStep, virtual public cScenarioExpSoccerGeneral
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioSoccerGeneralEval();
	virtual ~cScenarioSoccerGeneralEval();

	virtual std::string GetName() const;

	protected:
};
