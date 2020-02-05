#include "scenarios/DrawScenarioSoccerGeneralEval.h"
#include "scenarios/ScenarioSoccerGeneralEval.h"

#include "render/opengl.h"

cDrawScenarioSoccerGeneralEval::cDrawScenarioSoccerGeneralEval(cCamera& cam)
	: cDrawScenarioImitateStepEval(cam)
{
}

cDrawScenarioSoccerGeneralEval::~cDrawScenarioSoccerGeneralEval()
{
}

void cDrawScenarioSoccerGeneralEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioSoccerGeneralEval>(new cScenarioSoccerGeneralEval());
}

void cDrawScenarioSoccerGeneralEval::HandleRayTest(const cWorld::tRayTestResult& result)
{
#ifndef USE_OpenGLES
	if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
	{
		HandleRayTestBall(result);
	}
	else
#endif
	{
		cDrawScenarioImitateStepEval::HandleRayTest(result);
	}
}

void cDrawScenarioSoccerGeneralEval::HandleRayTestBall(const cWorld::tRayTestResult& result)
{
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeStatic)
		{
			SetBallPos(result.mHitPos);
		}
	}
}

void cDrawScenarioSoccerGeneralEval::DrawMisc() const
{
	cDrawScenarioImitateTargetEval::DrawMisc();
	DrawTargetPos();
}

void cDrawScenarioSoccerGeneralEval::SetBallPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioSoccerGeneralEval>(mScene);
	scene->SetBallPos(pos);
}

void cDrawScenarioSoccerGeneralEval::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->EnableRandTargetPos(true);
}
