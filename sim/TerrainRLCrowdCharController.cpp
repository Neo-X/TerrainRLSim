#include "TerrainRLCrowdCharController.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "MultiCharWaypointController.h"

//#define ENABLE_HEIGHT_MAP

const int cTerrainRLCrowdCharController::gVelSampleDim = 3;
const int gGroundVelFeatureDim = 3;

cTerrainRLCrowdCharController::cTerrainRLCrowdCharController() : cMultiCharWaypointController()
{
	mGroundOriginVel.setZero();
}

cTerrainRLCrowdCharController::~cTerrainRLCrowdCharController()
{
}

tVector cTerrainRLCrowdCharController::GetGroundVelSample(int s) const
{
	tVector vel = tVector(mGroundVelSamples(s, 0), mGroundVelSamples(s, 1), mGroundVelSamples(s, 2), 0);
	if (FlipStance())
	{
		vel[2] = -vel[2]; // reflect z
	}
	vel = mGroundSampleTrans * vel;
	vel += mGroundOriginVel;
	return vel;
}

void cTerrainRLCrowdCharController::InitGroundSamples()
{
	cMultiCharWaypointController::InitGroundSamples();

	int num_samples = GetNumGroundSamples();
	mGroundVelSamples = Eigen::MatrixXd::Zero(num_samples, gVelSampleDim);
}

void cTerrainRLCrowdCharController::ParseGround()
{
	mGroundSampleTrans = BuildGroundSampleTrans();

	tVector root_pos = mChar->GetRootPos();
	double ground_h = 0;
	tVector ground_vel = tVector::Zero();
	bool valid_sample = SampleGroundHeightVel(root_pos, ground_h, ground_vel);
	assert(valid_sample);
	mGroundOriginVel = ground_vel;

	if (HasGround())
	{
		SampleGround(mGroundSamples, mGroundVelSamples);
	}
	else
	{
		mGroundSamples.setZero();
		mGroundVelSamples.setZero();
	}
}

void cTerrainRLCrowdCharController::SampleGround(Eigen::VectorXd& out_h, Eigen::MatrixXd& out_vel) const
{
	tMatrix inv_trans = cMathUtil::InvRigidMat(mGroundSampleTrans);
	if (FlipStance())
	{
		inv_trans.row(2) *= -1; // reflect z
	}

	for (int i = 0; i < GetNumGroundSamples(); ++i)
	{
		tVector sample_pos = CalcGroundSamplePos(i);
		double h = 0;
		tVector vel = tVector::Zero();
		SampleGroundHeightVel(sample_pos, h, vel);

		h += inv_trans(1, 3);
		vel -= mGroundOriginVel;
		vel = inv_trans * vel;

		out_h[i] = h;
		out_vel.row(i) = vel.segment(0, gVelSampleDim);
	}
}

int cTerrainRLCrowdCharController::GetGroundFeatureSize() const
{
	int num_channels = gGroundVelFeatureDim;

	return num_channels * GetNumGroundSamples();
}

void cTerrainRLCrowdCharController::BuildPoliStateGround(Eigen::VectorXd& out_ground) const
{
	int num_samples = GetNumGroundSamples();
	out_ground.resize(GetGroundFeatureSize());

	out_ground.segment(0, num_samples) = mGroundVelSamples.col(0);
	out_ground.segment(num_samples, num_samples) = mGroundVelSamples.col(2);

	out_ground.segment(2 * num_samples, num_samples) = mGroundSamples;
}
