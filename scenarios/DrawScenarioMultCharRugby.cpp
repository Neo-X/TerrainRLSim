#include "DrawScenarioMultCharRugby.h"
#include "DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioMultCharRugby.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"
#include "sim/WaypointController.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"

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

void cDrawScenarioMultCharRugby::DrawTarget(const std::shared_ptr<cSimCharacter>& character) const
{
	cDrawScenarioMultChar::DrawTarget(character);

	auto scene = std::dynamic_pointer_cast<cScenarioMultCharRugby>(mScene);

	const auto& obj = scene->GetBall();
	cDrawUtil::SetColor(tVector(0.9,0.9,0.9,1));
	cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

	tVector line_col = GetLineColor();
	if (line_col[3] > 0)
	{
		cDrawUtil::SetColor(line_col);
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawWire);
	}

	cDrawUtil::SetColor(tVector(0.6,0.8,0.6,1));
	cDrawUtil::DrawBox(tVector(7.5,0,0,0), tVector(5,0.1,20,0), cDrawUtil::eDrawSolid);
	cDrawUtil::DrawBox(tVector(-7.5,0,0,0), tVector(5,0.1,20,0), cDrawUtil::eDrawSolid);



}


