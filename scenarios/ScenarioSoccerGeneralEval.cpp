#include "scenarios/ScenarioSoccerGeneralEval.h"

cScenarioSoccerGeneralEval::cScenarioSoccerGeneralEval() :
					cScenarioExpSoccerGeneral(),
					cScenarioImitateStepEval()
{
	EnableTargetPos(true);
	EnableRandTargetPos(true);
}

cScenarioSoccerGeneralEval::~cScenarioSoccerGeneralEval()
{
}

std::string cScenarioSoccerGeneralEval::GetName() const
{
	return "Soccer Evaluation";
}
