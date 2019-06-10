#pragma once
#include <memory>

#include "DrawScenarioSpaceMultChar.h"

class cDrawScenarioTestCaseMultChar : public cDrawScenarioSpaceMultChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTestCaseMultChar(cCamera& cam);
	virtual ~cDrawScenarioTestCaseMultChar();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:
	
	// bool mDrawOtherChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	// virtual void DrawCharacters() const;
	// virtual void DrawTarget(const std::shared_ptr<cSimCharacter>& character) const;
};
