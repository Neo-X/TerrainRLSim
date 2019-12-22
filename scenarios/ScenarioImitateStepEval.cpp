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

	tVector groundPosition = mChar->GetRootPos();
//	groundPosition[1] = 0.0;
//	tVector groundTargetNormal = (agent->GetCurrentGroundTarget() - groundPosition);
//	double disntace_g_a= groundTargetNormal.norm();

	double step_pose0 = (mStepPlan.mStepPos0 - groundPosition).norm();
	double step_pose1 = (mStepPlan.mStepPos1 - groundPosition).norm();

	// double step_pose0 = 	mStepPlan.mStepPos0.norm();
	// double step_pose1 = 	mStepPlan.mStepPos1.norm();
	if (step_pose0 >6.0 || step_pose1>6.0)
		return true;


	return false;
}

