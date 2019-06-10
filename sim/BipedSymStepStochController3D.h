#pragma once

#include "sim/CtStochController.h"
#include "sim/BipedSymStepController3D.h"

class cBipedSymStepStochController3D : public virtual cBipedSymStepController3D, 
										public virtual cCtStochController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedSymStepStochController3D();
	virtual ~cBipedSymStepStochController3D();

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual int GetNumNoiseUnits() const;
	virtual int GetNoiseStateOffset() const;

protected:

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);

	virtual int GetNoiseActionOffset() const;
};