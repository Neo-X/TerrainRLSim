#include "DrawScenarioSpaceMultChar.h"
#include "scenarios/ScenarioSpaceMultChar.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioSpaceMultChar::cDrawScenarioSpaceMultChar(cCamera& cam)
	: cDrawScenarioMultCharConcentricCircle(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioSpaceMultChar::~cDrawScenarioSpaceMultChar()
{

}

void cDrawScenarioSpaceMultChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioMultCharConcentricCircle::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		mDrawOtherChar = !mDrawOtherChar;
		break;
	default:
		break;
	}
}

void cDrawScenarioSpaceMultChar::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioSpaceMultChar>(new cScenarioSpaceMultChar());
}
