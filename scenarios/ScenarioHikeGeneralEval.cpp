#include "scenarios/ScenarioHikeGeneralEval.h"

cScenarioHikeGeneralEval::cScenarioHikeGeneralEval() :
					cScenarioExpSoccerGeneral(),
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
	return "Soccer General Evaluation";
}
