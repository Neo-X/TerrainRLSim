#pragma once
#include "DrawScenarioImitateEval.h"

class cKinCharacter;
class cDrawScenarioImitateEvalMultiTask : public cDrawScenarioImitateEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateEvalMultiTask(cCamera& cam);
	virtual ~cDrawScenarioImitateEvalMultiTask();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};
