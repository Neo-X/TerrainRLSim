#pragma once
#include <memory>

#include "DrawScenarioMultChar.h"

// class cDrawScenarioMultCharConcentricCircle : public cDrawScenarioPoliEval
class cDrawScenarioMultCharRugby : public cDrawScenarioMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMultCharRugby(cCamera& cam);
	virtual ~cDrawScenarioMultCharRugby();

	virtual void Keyboard(unsigned char key, int x, int y);
	// virtual std::string BuildTextInfoStr() const;
	virtual void ResetCallback();
	virtual void DrawTarget(const std::shared_ptr<cSimCharacter>& character) const;

protected:
	
	bool mDrawOtherChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};
