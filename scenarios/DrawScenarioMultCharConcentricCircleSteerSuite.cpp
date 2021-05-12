#include "DrawScenarioMultCharConcentricCircleSteerSuite.h"
#include "DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioMultCharConcentricCircleSteerSuite.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"
#include "sim/WaypointController.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioMultCharConcentricCircleSteerSuite::cDrawScenarioMultCharConcentricCircleSteerSuite(cCamera& cam)
	: cDrawScenarioMultChar(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioMultCharConcentricCircleSteerSuite::~cDrawScenarioMultCharConcentricCircleSteerSuite()
{

}

void cDrawScenarioMultCharConcentricCircleSteerSuite::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioMultCharConcentricCircleSteerSuite>(mScene);
	scene->EnableRandTargetPos(true);
}

void cDrawScenarioMultCharConcentricCircleSteerSuite::Keyboard(unsigned char key, int x, int y)
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

void cDrawScenarioMultCharConcentricCircleSteerSuite::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMultCharConcentricCircleSteerSuite>(new cScenarioMultCharConcentricCircleSteerSuite());
}


