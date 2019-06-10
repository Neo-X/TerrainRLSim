#pragma once


#include "MultiCharWaypointController.h"

class cTerrainRLCrowdCharVelController : public cMultiCharWaypointController
{
public:

	cTerrainRLCrowdCharVelController();
	virtual ~cTerrainRLCrowdCharVelController();

	virtual tVector GetGroundVelSample(int s) const;

	int GetNumGroundSamples() const;
	int GetGroundSampleRes() const;
	tVector CalcGroundSamplePos(int s) const;

protected:

	static const int gVelSampleDim;
	tVector mGroundOriginVel;
	Eigen::MatrixXd mGroundVelSamples;

	virtual void InitGroundSamples();
	virtual void ParseGround();
	virtual void SampleGround(Eigen::VectorXd& out_h, Eigen::MatrixXd& out_vel) const;

	virtual int GetGroundFeatureSize() const;
	virtual void BuildPoliStateGround(Eigen::VectorXd& out_ground) const;

};
