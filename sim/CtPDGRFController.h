#pragma once

#include "sim/CtPDController.h"
#include "sim/ImpPDController.h"
#include "sim/ExpPDController.h"

//#define ENABLE_EXP_PD_CTRL

class cCtPDGRFController : public virtual cCtPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPDGRFController();
	virtual ~cCtPDGRFController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:
	virtual void BuildPoliStateGRF(Eigen::VectorXd& out_pose) const;

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;

	virtual int GetGRFStateOffset() const;
	virtual size_t GetGRFStateSize() const;

};
