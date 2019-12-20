#include "ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpImitateStep.h"

cScenarioImitateStepEval::cScenarioImitateStepEval() :
	cScenarioExpImitateStep(),
	cScenarioImitateTargetEval()
{
	EnableTargetPos(false);
	EnableRandTargetPos(false);
}

cScenarioImitateStepEval::~cScenarioImitateStepEval()
{
}

std::string cScenarioImitateStepEval::GetName() const
{
	return "Imitate Step Evaluation";
}

bool cScenarioImitateStepEval::HasFallen() const
{
	bool fallen = cScenarioImitateTargetEval::HasFallen();
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	fallen = false;
#endif
	return fallen;
}

bool cScenarioImitateStepEval::endOfEpoch() const
{
	auto step_scene = std::dynamic_pointer_cast<cScenarioExpImitateStep>(mScene);
	const auto& step_plan = step_scene->GetStepPlan();
	double step_pose0 = step_plan.mStepPos0.norm();
	double step_pose1 = step_plan.mStepPos1.norm();

	if (step_pose0 >1.0 || step_pose1>1.0)
		return true;


	return false;
}

