#pragma once

#include "scenarios/ScenarioMultChar.h"

class cScenarioMultCharConcentricCircle : virtual public cScenarioMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMultCharConcentricCircle();
	virtual ~cScenarioMultCharConcentricCircle();

	virtual void Init();

	virtual std::string GetName() const;

protected:

	
	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t agentNum, double rand_rot);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void BuildChars();
	virtual void Reset();
};
