#pragma once

#include "scenarios/ScenarioExpImitateStep.h"
#include "sim/Ground.h"
#include "sim/BipedStepController3D.h"
#include "anim/KinController.h"

//#define ENABLE_KIN_CONTROLLER_TEST

class cScenarioExpImitateStepHumanoid : virtual public cScenarioExpImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	cScenarioExpImitateStepHumanoid();
	virtual ~cScenarioExpImitateStepHumanoid();

	std::string GetName() const;


protected:

	virtual void SetupKinController();

};
