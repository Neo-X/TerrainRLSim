#pragma once
#include <memory>

#include "DrawScenarioMultCharConcentricCircle.h"

class cDrawScenarioSpaceMultChar : public cDrawScenarioMultCharConcentricCircle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioSpaceMultChar(cCamera& cam);
	virtual ~cDrawScenarioSpaceMultChar();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:
	
	// bool mDrawOtherChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	// virtual void DrawCharacters() const;
	// virtual void DrawTarget(const std::shared_ptr<cSimCharacter>& character) const;
};
