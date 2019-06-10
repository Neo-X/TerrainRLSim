#include "DrawScenarioTestCaseMultChar.h"
#include "scenarios/ScenarioTestCaseMultChar.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioTestCaseMultChar::cDrawScenarioTestCaseMultChar(cCamera& cam)
	: cDrawScenarioSpaceMultChar(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioTestCaseMultChar::~cDrawScenarioTestCaseMultChar()
{

}

void cDrawScenarioTestCaseMultChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSpaceMultChar::Keyboard(key, x, y);

	switch (key)
	{
	default:
		break;
	}
}

void cDrawScenarioTestCaseMultChar::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTestCaseMultChar>(new cScenarioTestCaseMultChar());
}
