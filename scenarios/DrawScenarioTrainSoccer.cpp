#include "DrawScenarioTrainSoccer.h"
#include "scenarios/ScenarioTrainSoccer.h"
#include "scenarios/ScenarioExpSoccer.h"

#include "render/opengl.h"

cDrawScenarioTrainSoccer::cDrawScenarioTrainSoccer(cCamera& cam)
	: cDrawScenarioTrainHike(cam)
{
}

cDrawScenarioTrainSoccer::~cDrawScenarioTrainSoccer()
{
}

void cDrawScenarioTrainSoccer::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainSoccer>(new cScenarioTrainSoccer());
}


void cDrawScenarioTrainSoccer::HandleRayTest(const cWorld::tRayTestResult& result)
{
#ifndef USE_OpenGLES
	if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
	{
		HandleRayTestBall(result);
	}
	else
#endif
	{
		cDrawScenarioTrainHike::HandleRayTest(result);
	}
}

void cDrawScenarioTrainSoccer::HandleRayTestBall(const cWorld::tRayTestResult& result)
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

void cDrawScenarioTrainSoccer::SetBallPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpSoccer>(mScene);
	scene->SetBallPos(pos);
}
