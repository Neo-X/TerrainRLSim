#pragma once
#include "scenarios/ScenarioImitateEval.h"
#include "anim/KinCharacter.h"

class cScenarioImitateEvalMultiTask : virtual public cScenarioImitateEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateEvalMultiTask();
	virtual ~cScenarioImitateEvalMultiTask();

	virtual void ParseGroundParams(const std::shared_ptr<cArgParser>& parser, cGround::tParams& out_params) const;

protected:

	virtual bool LoadTerrains(const Json::Value& json, cGround::tParams& out_params) const;
	virtual void LoadTerrains(const std::vector<std::string>& motion_files, cGround::tParams& out_params) const;


};
