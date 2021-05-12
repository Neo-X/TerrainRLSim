#include "DrawScenarioMultCharConcentricCircle.h"
#include "DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioMultCharConcentricCircle.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"
#include "sim/WaypointController.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioMultCharConcentricCircle::cDrawScenarioMultCharConcentricCircle(cCamera& cam)
	: cDrawScenarioMultChar(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioMultCharConcentricCircle::~cDrawScenarioMultCharConcentricCircle()
{

}

void cDrawScenarioMultCharConcentricCircle::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioMultCharConcentricCircle>(mScene);
	scene->EnableRandTargetPos(true);
}

void cDrawScenarioMultCharConcentricCircle::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioMultChar::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		mDrawOtherChar = !mDrawOtherChar;
		break;
	default:
		break;
	}
}

void cDrawScenarioMultCharConcentricCircle::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMultCharConcentricCircle>(new cScenarioMultCharConcentricCircle());
}


