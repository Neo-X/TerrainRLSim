#include "scenarios/ScenarioExpImitateStepHumanoid.h"
#include "sim/RBDUtil.h"
#include "anim/MocapStepController.h"
#include "anim/MotionFieldStepController.h"

#define ENABLE_STEP_FAIL
//#define DISABLE_FOOTSTEP_REWARD
//#define ENABLE_IND_POSE_ERR
//#define ENABLE_MOTION_FIELD
#define SAMPLE_INIT_STATES

//#define ENABLE_STYLE_MARCH
//#define ENABLE_STYLE_LEAN
//#define ENABLE_STYLE_SIDE_LEAN
//#define ENABLE_STYLE_STRAIGHT_KNEE_RIGHT
//#define ENABLE_STYLE_STRAIGHT_KNEE_LEFT

//#define ENABLE_ADAPTIVE_STEP_PLAN



cScenarioExpImitateStepHumanoid::cScenarioExpImitateStepHumanoid() :
	cScenarioExpImitateStep()
{

}

cScenarioExpImitateStepHumanoid::~cScenarioExpImitateStepHumanoid()
{
}


std::string cScenarioExpImitateStepHumanoid::GetName() const
{
	return "Humanoid Imitate Step Exploration";
}

void cScenarioExpImitateStepHumanoid::SetupKinController()
{
	std::shared_ptr<cKinController> kin_ctrl;
	BuildKinController(kin_ctrl);
	
	auto step_ctrl = std::dynamic_pointer_cast<cMocapStepController>(kin_ctrl);
	if (step_ctrl != nullptr)
	{
		// order matters!
		step_ctrl->setRelativeFilePath(this->getRelativeFilePath());
		step_ctrl->SetFootJoints(mEndEffectors[cBipedStepController3D::eStanceRight],
								mEndEffectors[cBipedStepController3D::eStanceLeft]);
		step_ctrl->SetCyclePeriod(mCtrlParams.mCycleDur);
		step_ctrl->Init(mKinChar.get(), mKinCtrlFile);
	}

	mKinChar->SetController(kin_ctrl);
}

