#pragma once

#include "scenarios/ScenarioHikeEval.h"
#include "scenarios/ScenarioExpSoccer.h"
#include "scenarios/ScenarioExpHike.h"

class cScenarioSoccerEval : virtual public cScenarioHikeEval, virtual public cScenarioExpHike, virtual public cScenarioExpSoccer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioSoccerEval();
	virtual ~cScenarioSoccerEval();

	virtual std::string GetName() const;

protected:
};
