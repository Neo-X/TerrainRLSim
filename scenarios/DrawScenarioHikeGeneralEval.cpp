#include "scenarios/DrawScenarioHikeGeneralEval.h"

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
	out_scene = std::shared_ptr<cScenarioHikeGeneralEval>(new cScenarioHikeGeneralEval());
}

void cDrawScenarioHikeGeneralEval::HandleRayTest(const cWorld::tRayTestResult& result)
{
#ifndef USE_OpenGLES
	if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
	{
		// HandleRayTestBall(result);
	}
	else
#endif
	{
		cDrawScenarioImitateStepEval::HandleRayTest(result);
	}
}

void cDrawScenarioHikeGeneralEval::DrawMisc() const
{
	cDrawScenarioImitateTargetEval::DrawMisc();
	DrawTargetPos();
}

void cDrawScenarioHikeGeneralEval::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->EnableRandTargetPos(true);
}
