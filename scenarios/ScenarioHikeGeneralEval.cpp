#include "scenarios/ScenarioHikeGeneralEval.h"

cScenarioHikeGeneralEval::cScenarioHikeGeneralEval() :
					cScenarioExpHikeGeneral(),
					cScenarioImitateStepEval()
{
	EnableTargetPos(true);
	EnableRandTargetPos(true);
}

cScenarioHikeGeneralEval::~cScenarioHikeGeneralEval()
{
}

std::string cScenarioHikeGeneralEval::GetName() const
{
	return "Hike General Evaluation";
}
