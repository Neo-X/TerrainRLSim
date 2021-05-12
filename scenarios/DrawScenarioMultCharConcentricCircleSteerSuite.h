#pragma once
#include <memory>

#include "DrawScenarioMultChar.h"

// class cDrawSteerSuiteScenarioMultCharConcentricCircle : public cDrawScenarioPoliEval
class cDrawScenarioMultCharConcentricCircleSteerSuite : public cDrawScenarioMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMultCharConcentricCircleSteerSuite(cCamera& cam);
	virtual ~cDrawScenarioMultCharConcentricCircleSteerSuite();

	virtual void Keyboard(unsigned char key, int x, int y);
	// virtual std::string BuildTextInfoStr() const;
	virtual void ResetCallback();

protected:
	
	bool mDrawOtherChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};
