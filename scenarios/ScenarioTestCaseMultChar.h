#pragma once

#include "scenarios/ScenarioSpaceMultChar.h"

class cScenarioTestCaseMultChar : virtual public cScenarioSpaceMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTestCaseMultChar();
	virtual ~cScenarioTestCaseMultChar();

	virtual void Init();

	virtual std::string GetName() const;

	virtual void Reset();

protected:

	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t agentNum, double rand_rot);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	// virtual void Reset();
};
