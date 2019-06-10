#pragma once

#include "scenarios/ScenarioMultCharConcentricCircle.h"

class cScenarioSpaceMultChar : virtual public cScenarioMultCharConcentricCircle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioSpaceMultChar();
	virtual ~cScenarioSpaceMultChar();

	virtual void Init();

	virtual std::string GetName() const;

	virtual void Reset();

protected:

	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t agentNum, double rand_rot);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	// virtual void Reset();
};
