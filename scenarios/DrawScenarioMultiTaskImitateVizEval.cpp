#include "DrawScenarioMultiTaskImitateVizEval.h"
#include "scenarios/ScenarioMultiTaskImitateVizEval.h"
#include "anim/KinCharacter.h"
#include "anim/KinSimCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(1, 1, 1, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioMultiTaskImitateVizEval::cDrawScenarioMultiTaskImitateVizEval(cCamera& cam)
	: cDrawScenarioImitateVizEval(cam)
{
	mDrawKinChar = true;
	mTrackKinChar = false;
}

cDrawScenarioMultiTaskImitateVizEval::~cDrawScenarioMultiTaskImitateVizEval()
{
}

void cDrawScenarioMultiTaskImitateVizEval::Init()
{
	cDrawScenarioImitateVizEval::Init();
}

void cDrawScenarioMultiTaskImitateVizEval::Reset()
{
	cDrawScenarioImitateVizEval::Reset();
}

void cDrawScenarioMultiTaskImitateVizEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMultiTaskImitateVizEval>(new cScenarioMultiTaskImitateVizEval());
}

