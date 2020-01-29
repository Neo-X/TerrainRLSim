#include "scenarios/ScenarioSoccerGeneralEval.h"

cScenarioSoccerGeneralEval::cScenarioSoccerGeneralEval() :
					cScenarioExpSoccerGeneral(),
					cScenarioExpImitateStep()
{
}

cScenarioSoccerGeneralEval::~cScenarioSoccerGeneralEval()
{
}

std::string cScenarioSoccerGeneralEval::GetName() const
{
	return "Soccer Evaluation";
}
