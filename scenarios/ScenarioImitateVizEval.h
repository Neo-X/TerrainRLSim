#pragma once

#include "scenarios/ScenarioPoliEval.h"
#include "scenarios/ScenarioExpImitateViz.h"
#include "anim/KinCharacter.h"

class cScenarioImitateVizEval : virtual public cScenarioPoliEval, virtual public cScenarioExpImitateViz
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateVizEval();
	virtual ~cScenarioImitateVizEval();

	virtual void GetPoseErrResult(double& out_err, int& out_count) const;

	virtual void ResetRecord();
	virtual void SetResetPhase(double phase_min, double phase_max);
	virtual void SetResetPhaseSamples(int num_samples);

	virtual std::string GetName() const;

protected:

	double mResetPhaseMin;
	double mResetPhaseMax;
	int mResetPhaseSamples;

	bool mRecordPoseError;
	double mPoseErr;
	int mPoseErrCount;

	virtual void UpdatePoseErr();
	virtual void RecordAction(const std::string& out_file);
	virtual double CalcRandKinResetTime();

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void InitMisc();
	virtual void ResetMisc();
	virtual void ClearMisc();
	virtual void UpdateMisc(double time_elapsed);
};
