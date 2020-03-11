#include "scenarios/DrawScenarioHikeGeneralEval.h"
#include "scenarios/ScenarioHikeGeneralEval.h"

#include "render/opengl.h"

cDrawScenarioHikeGeneralEval::cDrawScenarioHikeGeneralEval(cCamera& cam)
	: cDrawScenarioImitateStepEval(cam)
{
}

cDrawScenarioHikeGeneralEval::~cDrawScenarioHikeGeneralEval()
{
}

void cDrawScenarioHikeGeneralEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioSoccerGeneralEval>(new cScenarioSoccerGeneralEval());
}

void cDrawScenarioHikeGeneralEval::HandleRayTest(const cWorld::tRayTestResult& result)
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

void cDrawScenarioHikeGeneralEval::HandleRayTestBall(const cWorld::tRayTestResult& result)
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

void cDrawScenarioHikeGeneralEval::DrawMisc() const
{
	cDrawScenarioImitateTargetEval::DrawMisc();
	DrawTargetPos();
}

void cDrawScenarioHikeGeneralEval::SetBallPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioSoccerGeneralEval>(mScene);
	scene->SetBallPos(pos);
}

void cDrawScenarioHikeGeneralEval::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->EnableRandTargetPos(true);
}
