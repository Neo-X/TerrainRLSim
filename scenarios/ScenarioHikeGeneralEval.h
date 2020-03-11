#pragma once

#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpHikeGeneral.h"

class cScenarioHikeGeneralEval : virtual public cScenarioImitateStepEval, virtual public cScenarioExpHikeGeneral
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioHikeGeneralEval();
	virtual ~cScenarioHikeGeneralEval();

	virtual std::string GetName() const;

	protected:

};
