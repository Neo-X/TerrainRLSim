#include "ScenarioMultiTaskImitateVizEval.h"
#include "scenarios/ScenarioExpImitateViz.h"
#include "sim/CtTrackController.h"
#include "sim/CtPhaseController.h"
#include "sim/WaypointController.h"
#include "sim/SimCharSoftFall.h"
#include "util/FileUtil.h"
#include "anim/MocapDataBaseController.h"

cScenarioMultiTaskImitateVizEval::cScenarioMultiTaskImitateVizEval() :
	cScenarioImitateVizEval()
{

	// EnableTargetPos(true);
}

cScenarioMultiTaskImitateVizEval::~cScenarioMultiTaskImitateVizEval()
{
}

void cScenarioMultiTaskImitateVizEval::Init()
{
	cScenarioImitateVizEval::Init();

	// InitEndEffectors();
	SetupKinController();
}

std::string cScenarioMultiTaskImitateVizEval::GetName() const
{
	return "MultiTask Visual Imitation.";
}

void cScenarioMultiTaskImitateVizEval::SetupKinController()
{
	std::shared_ptr<cKinController> kin_ctrl;
	BuildKinController(kin_ctrl);

	auto step_ctrl = std::dynamic_pointer_cast<cMocapDataBaseController>(kin_ctrl);
	if (step_ctrl != nullptr)
	{
		// order matters!
		step_ctrl->setRelativeFilePath(this->getRelativeFilePath());
		// step_ctrl->SetFootJoints(mEndEffectors[cBipedStepController3D::eStanceRight],
		//						mEndEffectors[cBipedStepController3D::eStanceLeft]);
		step_ctrl->SetCyclePeriod(mCtrlParams.mCycleDur);
		step_ctrl->Init(mKinChar.get(), mKinCtrlFile);
	}

	mKinChar->SetController(kin_ctrl);
}

void cScenarioMultiTaskImitateVizEval::BuildKinController(std::shared_ptr<cKinController>& out_ctrl) const
{
	auto ctrl = std::shared_ptr<cMocapDataBaseController>(new cMocapDataBaseController);
	//ctrl->EnableAutoStepUpdate(false);
	out_ctrl = ctrl;
}

void cScenarioMultiTaskImitateVizEval ::SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const
{
	cScenarioExpCacla::SetupControllerParams(out_params);
}

void cScenarioMultiTaskImitateVizEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioImitateVizEval::ParseArgs(parser);
	parser->ParseString("kin_ctrl_file", mKinCtrlFile);

}

void cScenarioMultiTaskImitateVizEval::InitEndEffectors()
{
	mEndEffectors.clear();
	int num_joints = mChar->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		if (mChar->IsEndEffector(j))
		{
			mEndEffectors.push_back(j);
		}
	}
	assert(mEndEffectors.size() == cBipedStepController3D::eStanceMax);
}

size_t cScenarioMultiTaskImitateVizEval::getMotionID() const
{
	return mKinChar->GetController()->getMotionID();
}

size_t cScenarioMultiTaskImitateVizEval::GetNumMotions() const
{
	return mKinChar->GetController()->GetNumMotions();
}

void cScenarioMultiTaskImitateVizEval::setMotionID(size_t task)
{
	return mKinChar->GetController()->setMotionID(task);
}
