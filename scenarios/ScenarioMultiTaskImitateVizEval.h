#pragma once

#include "scenarios/ScenarioImitateVizEval.h"
#include "scenarios/ScenarioExpImitateStep.h"
#include "anim/KinCharacter.h"
#include "anim/KinController.h"

class cScenarioMultiTaskImitateVizEval : virtual public cScenarioImitateVizEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMultiTaskImitateVizEval();
	virtual ~cScenarioMultiTaskImitateVizEval();

	virtual void Init();

	virtual std::string GetName() const;
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual size_t getMotionID() const;
	virtual void setMotionID(size_t task);
	virtual size_t GetNumMotions() const;

protected:

	std::vector<int> mEndEffectors;
	std::string mKinCtrlFile;


	virtual void SetupKinController();
	virtual void BuildKinController(std::shared_ptr<cKinController>& out_ctrl) const;
	virtual void SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const;

	virtual void InitEndEffectors();
};
