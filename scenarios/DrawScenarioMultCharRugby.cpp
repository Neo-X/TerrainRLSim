#include "DrawScenarioMultCharRugby.h"
#include "DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioMultCharRugby.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"
#include "sim/WaypointController.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioMultCharRugby::cDrawScenarioMultCharRugby(cCamera& cam)
	: cDrawScenarioMultChar(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioMultCharRugby::~cDrawScenarioMultCharRugby()
{

}

void cDrawScenarioMultCharRugby::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioMultCharRugby>(mScene);
	scene->EnableRandTargetPos(true);
}

void cDrawScenarioMultCharRugby::Keyboard(unsigned char key, int x, int y)
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

void cDrawScenarioMultCharRugby::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMultCharRugby>(new cScenarioMultCharRugby());
}


