#pragma once
#include "DrawScenarioPoliEval.h"

class cKinCharacter;
class cDrawScenarioImitateVizEval : public cDrawScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateVizEval(cCamera& cam);
	virtual ~cDrawScenarioImitateVizEval();

	virtual void Init();
	virtual void Reset();
	virtual void Keyboard(unsigned char key, int x, int y);

	virtual void Draw();
	virtual void DrawMisc() const;
	virtual void DrawTargetPos() const;
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:

	bool mDrawKinChar;
	bool mTrackKinChar;
	double zoom;
	double cam_height;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawKinChar() const;
	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual void ToggleDrawKinChar();
	virtual void ToggleTrackKinCharacter();
	virtual tVector GetDrawKinCharOffset() const;

	virtual tVector GetCamTrackPos() const;
};
