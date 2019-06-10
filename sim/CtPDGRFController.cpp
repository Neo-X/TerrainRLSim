#include "CtPDGRFController.h"
#include "sim/SimCharacter.h"

cCtPDGRFController::cCtPDGRFController() : cCtPDController()
{
}

cCtPDGRFController::~cCtPDGRFController()
{
}

void cCtPDGRFController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtPDController::Init(character, gravity, param_file);
}

void cCtPDGRFController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	int state_size = GetPoliStateSize() + GetGRFStateSize();
	Eigen::VectorXd out_state2(state_size);

	cCtPDController::BuildPoliState(out_state2);
	out_state.resize(state_size);
	Eigen::VectorXd grf_state;
	this->BuildPoliStateGRF(grf_state);


	int grf_offset = GetGRFStateOffset();
	int grf_size = GetGRFStateSize();
	// TODO: In debug mode ground has 200 samples in it that EIGEN will complain about assigning to a vector of size 0.
	out_state.segment(0, GetPoliStateSize()) = out_state2;
	out_state.segment(grf_offset, grf_size) = grf_state;
}

int cCtPDGRFController::GetGRFStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

size_t cCtPDGRFController::GetGRFStateSize() const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	size_t eofs = 0;
	int num_parts = mChar->GetNumBodyParts();

	int pos_dim = GetPosFeatureDim();

	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			if (mChar->IsEndEffector(part_id))
			{
				eofs++;
			}
		}
	}
	return eofs * pos_dim;
}

void cCtPDGRFController::BuildPoliStateGRF(Eigen::VectorXd& out_impuse) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::VectorXd& pose = mChar->GetPose();
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	tVector root_pos = mChar->GetRootPos();
	tVector root_pos_rel = root_pos;

	out_impuse = Eigen::VectorXd::Zero(GetGRFStateSize());
	int num_parts = mChar->GetNumBodyParts();

	int pos_dim = GetPosFeatureDim();

	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			if (mChar->IsEndEffector(part_id))
			{
				tVector curr_impulse = curr_part->GetContactImpulse();
				// std::cout << "current link: " << part_id << " contact impulse: " << curr_impulse << std::endl;

				out_impuse.segment(idx, pos_dim) = curr_impulse.segment(0, pos_dim);
				idx += pos_dim;

			}
		}
	}
}

