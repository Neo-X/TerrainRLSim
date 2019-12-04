#pragma once
#include "scenarios/ScenarioImitateEval.h"
#include "anim/KinCharacter.h"

class cScenarioImitateEvalMultiTask : virtual public cScenarioImitateEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateEvalMultiTask();
	virtual ~cScenarioImitateEvalMultiTask();

	virtual void setTaskID(size_t task);
	virtual size_t getTaskID() const;
	virtual size_t GetNumTasks() const;

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:

	std::vector<cGround::tParams> groundParams;

	virtual void ParseGroundParams(const std::shared_ptr<cArgParser>& parser, std::vector<cGround::tParams>& out_params) const;
	virtual bool LoadTerrains(const Json::Value& json, std::vector<cGround::tParams>& out_params) const;
	virtual void LoadTerrains(const std::vector<std::string>& motion_files, std::vector<cGround::tParams>& out_params) const;

};
