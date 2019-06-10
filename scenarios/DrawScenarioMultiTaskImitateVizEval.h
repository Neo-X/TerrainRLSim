#pragma once
#include "DrawScenarioImitateVizEval.h"

class cDrawScenarioMultiTaskImitateVizEval : public cDrawScenarioImitateVizEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMultiTaskImitateVizEval(cCamera& cam);
	virtual ~cDrawScenarioMultiTaskImitateVizEval();

	virtual void Init();
	virtual void Reset();

	void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;

};
