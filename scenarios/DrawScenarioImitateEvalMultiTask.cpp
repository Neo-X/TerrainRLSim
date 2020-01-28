#include "DrawScenarioImitateEvalMultiTask.h"
#include "scenarios/ScenarioImitateEvalMultiTask.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6, 0.65, 0.675, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioImitateEvalMultiTask::cDrawScenarioImitateEvalMultiTask(cCamera& cam)
	: cDrawScenarioImitateEval(cam)
{
	mDrawKinChar = true;
}

cDrawScenarioImitateEvalMultiTask::~cDrawScenarioImitateEvalMultiTask()
{
}

void cDrawScenarioImitateEvalMultiTask::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateEvalMultiTask>(new cScenarioImitateEvalMultiTask());
}

