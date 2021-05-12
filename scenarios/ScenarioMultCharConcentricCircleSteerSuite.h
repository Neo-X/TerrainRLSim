#pragma once

#include "scenarios/ScenarioMultChar.h"
#include "tinyxml2.h"

class cScenarioMultCharConcentricCircleSteerSuite : virtual public cScenarioMultChar
{
struct location{
    double x;
    double y;
    double z;
};
struct agent {
   double radius;
   struct location position;
   struct location direction;
   double speed;
   struct location targetLocation;
   double desiredSpeed;
   double timeDuration;
};


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	

	cScenarioMultCharConcentricCircleSteerSuite();
	virtual ~cScenarioMultCharConcentricCircleSteerSuite();

	virtual void Init();

	virtual std::string GetName() const;

protected:
    std::string steersuite_file;
	std::vector<agent> agents;
	virtual void GenerateInitialTransform(std::shared_ptr<cSimCharacter>& character, size_t agentNum, double rand_rot);
	virtual tVector CalcTargetPosObstaclesDynamicCharacters3D(std::shared_ptr<cSimCharacter>& character);

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void BuildChars();
	virtual void Reset();
};
