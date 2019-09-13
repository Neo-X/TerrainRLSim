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

void cDrawScenarioMultChar::DrawTarget(const std::shared_ptr<cSimCharacter>& character) const
{
	cDrawScenarioMultChar::DrawTarget(character);
	/*
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(character->GetCurrentGroundTarget());
	cDrawUtil::DrawSphere( 0.25 );
	cDrawUtil::PopMatrix();

	cDrawUtil::DrawLine( groundProjectedPosition, character->GetCurrentGroundTarget() );
	*/
	tVector groundProjectedPosition = tVector(character->GetRootPos()[0], 0, character->GetRootPos()[2], 0);
	const double r = 0.1;
	const double line_h = 2;
	const double line_w = 2;
	const tVector col = tVector(1, 0, 0, 0.5);

	auto scene = std::dynamic_pointer_cast<cScenarioMultCharRugby>(mScene);
	const tVector& target_pos = character->GetCurrentGroundTarget();
	double reset_dist = scene->GetTargetResetDist();

	cDrawUtil::PushMatrix();

	cDrawUtil::Translate(target_pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);

	cDrawUtil::PopMatrix();
	// cDrawUtil::SetColor(tVector(0.2, 1.0, 0.2, 0.5));
	cDrawUtil::DrawLine( groundProjectedPosition, character->GetCurrentGroundTarget() );
}


