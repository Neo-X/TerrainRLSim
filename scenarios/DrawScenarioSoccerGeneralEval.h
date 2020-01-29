#pragma once
#include "scenarios/DrawScenarioImitateTargetEval.h"

class cDrawScenarioSoccerGeneralEval : public cDrawScenarioImitateTargetEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioSoccerGeneralEval(cCamera& cam);
	virtual ~cDrawScenarioSoccerGeneralEval();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void HandleRayTestBall(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);
};
