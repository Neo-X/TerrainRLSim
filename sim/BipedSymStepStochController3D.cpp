#include "BipedSymStepStochController3D.h"

cBipedSymStepStochController3D::cBipedSymStepStochController3D() : 
									cBipedSymStepController3D(),
									cCtStochController()
{
}

cBipedSymStepStochController3D::~cBipedSymStepStochController3D()
{
}

int cBipedSymStepStochController3D::GetPoliStateSize() const
{
	int state_size = cBipedSymStepController3D::GetPoliStateSize();
	state_size += GetNumNoiseUnits();
	return state_size;
}

int cBipedSymStepStochController3D::GetPoliActionSize() const
{
	int size = cBipedSymStepController3D::GetPoliActionSize();
	size += GetNumNoiseUnits();
	return size;
}

void cBipedSymStepStochController3D::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cBipedSymStepController3D::BuildNNInputOffsetScaleTypes(out_types);

	int offset = GetNoiseStateOffset();
	int size = GetNoiseStateSize();
	for (int i = 0; i < size; ++i)
	{
		out_types[offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

void cBipedSymStepStochController3D::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cBipedSymStepController3D::BuildActorOutputOffsetScale(out_offset, out_scale);

	int noise_offset = GetNoiseActionOffset();
	int noise_size = GetNoiseActionSize();
	out_offset.segment(noise_offset, noise_size) = Eigen::VectorXd::Zero(noise_size);
	out_scale.segment(noise_offset, noise_size) = Eigen::VectorXd::Ones(noise_size);
}

void cBipedSymStepStochController3D::GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	cBipedSymStepController3D::GetPoliActionBounds(out_min, out_max);

	Eigen::VectorXd noise_min;
	Eigen::VectorXd noise_max;
	BuildActionNoiseBounds(noise_min, noise_max);

	int noise_offset = GetNoiseActionOffset();
	int noise_size = GetNoiseActionSize();
	out_min.segment(noise_offset, noise_size) = noise_min;
	out_max.segment(noise_offset, noise_size) = noise_max;
}

void cBipedSymStepStochController3D::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cBipedSymStepController3D::BuildPoliState(out_state);

	int noise_offset = GetNoiseStateOffset();
	int noise_size = GetNoiseStateSize();
	out_state.segment(noise_offset, noise_size).setZero();
}

void cBipedSymStepStochController3D::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
	ApplyExpNoiseInternal(state);
	cBipedSymStepController3D::ExploreAction(state, out_action);
}

int cBipedSymStepStochController3D::GetNumNoiseUnits() const
{
	int num_units = mNet->GetInputSize() - cBipedSymStepController3D::GetPoliStateSize();
	num_units = std::max(0, num_units);
	return num_units;
}

int cBipedSymStepStochController3D::GetNoiseStateOffset() const
{
	return cBipedSymStepController3D::GetPoliStateSize();
}

int cBipedSymStepStochController3D::GetNoiseActionOffset() const
{
	return cBipedSymStepController3D::GetPoliActionSize();
}