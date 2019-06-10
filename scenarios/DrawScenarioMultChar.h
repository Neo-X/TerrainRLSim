#pragma once
#include <memory>

#include "DrawScenarioPoliEval.h"
#include "DrawScenarioHikeEval.h"

// class cDrawScenarioMultChar : public cDrawScenarioPoliEval
class cDrawScenarioMultChar : public cDrawScenarioHikeEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMultChar(cCamera& cam);
	virtual ~cDrawScenarioMultChar();

	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void SetTargetPos(const tVector& pos);
	// virtual std::string BuildTextInfoStr() const;
	virtual void ResetCallback();
	void DrawStepPlan() const;

protected:
	
	bool mDrawOtherChar;
	std::vector<cCharTracer> mTracers;

	virtual void UpdateTracer(double time_elapsed);
	virtual void InitTracer();

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawTarget(const std::shared_ptr<cSimCharacter>& character) const;
};
