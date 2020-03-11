#pragma once
#include "scenarios/DrawScenarioImitateStepEval.h"

class cDrawScenarioHikeGeneralEval : public cDrawScenarioImitateStepEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioHikeGeneralEval(cCamera& cam);
	virtual ~cDrawScenarioHikeGeneralEval();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void HandleRayTestBall(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);
	virtual void ResetCallback();
	virtual void DrawMisc() const;
};
