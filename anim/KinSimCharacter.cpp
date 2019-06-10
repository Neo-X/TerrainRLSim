#include "KinSimCharacter.h"
#include <assert.h>
#include <functional>
#include <iostream>

#include "sim/SimBox.h"
#include "sim/SimCapsule.h"
#include "sim/SimSphere.h"
#include "sim/SimCylinder.h"
#include "sim/RBDUtil.h"

const double gDiffTimeStep = 1 / 60.0;

cKinSimCharacter::cKinSimCharacter()
{
	mOrigin.setZero();
	mOriginRot.setIdentity();
	mCycleRootDelta.setZero();
	mEnableVelUpdate = true;
	charFile = "";
	timeWarping = 1.0;
}

cKinSimCharacter::~cKinSimCharacter()
{

}

bool cKinSimCharacter::Init(const std::string& char_file,
		const std::string& motion_file)
{
	this->charFile = char_file;
	bool succ = Init(char_file);
	if (succ && (motion_file != ""))
	{
		bool succ_motion = LoadMotion(motion_file);
		if (!succ_motion)
		{
			printf("Failed to load motion from %s\n", motion_file.c_str());
		}
		succ &= succ_motion;
	}

	return succ;
}

bool cKinSimCharacter::BuildBody(std::shared_ptr<cWorld> world)
{
	mWorld = world;

	bool succ_body = true;
	const tVector& root_pos = tVector(0, 0, 0, 0);
	succ_body = BuildSimBody(this->charFile, root_pos);
	cKinTree::LoadDrawShapeDefs(this->charFile, mDrawShapeDefs);
	// LoadDrawShapeDefs(char_file, mDrawShapeDefs);
	return succ_body;
}

bool cKinSimCharacter::Init(const std::string& char_file)
{
	return cCharacter::Init(char_file);
}

void cKinSimCharacter::Clear()
{
	cCharacter::Clear();
	mMotion.Clear();

	if (HasController())
	{
		mController->Clear();
	}
}

void cKinSimCharacter::Update(double time_step)
{
	cCharacter::Update(time_step);
	mTime += time_step * timeWarping;

	if (HasController())
	{
		mController->Update(time_step);
	}
	
	Pose(mTime);
	/// Updating sim body velocities
	updateBodyLinkVelocities();
}

void cKinSimCharacter::updateBodyLinkVelocities()
{

	if (mEnableVelUpdate)
	{

		double time_ = mTime;
		Eigen::VectorXd pose0 = GetPose();
		Eigen::VectorXd pose1;
		CalcPose(mTime + gDiffTimeStep, pose1);
		// cKinTree::CalcVel(mJointMat, pose, pose1, gDiffTimeStep, mVel);
		/// need positions of body links

		assert(HasDrawShapes());
		const auto& shape_defs = GetDrawShapeDefs();
		size_t num_shapes = mBodyDefs.rows();

		// cDrawUtil::SetLineWidth(1);
		std::vector<tVector> link_positions;
		for (int i = 0; i < num_shapes; ++i)
		{
			cKinTree::tDrawShapeDef curr_def = shape_defs.row(i);
			// cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
			tVector euler = cKinTree::GetDrawShapeAttachTheta(curr_def);
			int parent_joint = cKinTree::GetDrawShapeParentJoint(curr_def);
			tVector attach_pt = cKinTree::GetDrawShapeAttachPt(curr_def);
			tMatrix world_trans = BuildJointWorldTrans(parent_joint);
			world_trans = world_trans * cMathUtil::TranslateMat(attach_pt) * cMathUtil::RotateMat(euler);
			tVector pos_ = tVector(world_trans(0,3), world_trans(1,3), world_trans(2,3), 0);
			link_positions.push_back(pos_);
		}

		/// Link positions for the future
		Pose(mTime + gDiffTimeStep);
		for (int i = 0; i < num_shapes; ++i)
		{
			cKinTree::tDrawShapeDef curr_def = shape_defs.row(i);
			// cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
			tVector euler = cKinTree::GetDrawShapeAttachTheta(curr_def);
			int parent_joint = cKinTree::GetDrawShapeParentJoint(curr_def);
			tVector attach_pt = cKinTree::GetDrawShapeAttachPt(curr_def);
			tMatrix world_trans = BuildJointWorldTrans(parent_joint);
			world_trans = world_trans * cMathUtil::TranslateMat(attach_pt) * cMathUtil::RotateMat(euler);
			tVector pos_ = tVector(world_trans(0,3), world_trans(1,3), world_trans(2,3), 0);
			tVector vel_ = (pos_ - link_positions[i]) / gDiffTimeStep;
			this->SetBodyPartVel(i, vel_);
		}
		Pose(mTime);
	}
}

void cKinSimCharacter::Reset()
{
	cCharacter::Reset();
	if (HasController())
	{
		mController->Reset();
	}
}

bool cKinSimCharacter::BuildSimBody(const std::string& char_file, const tVector& root_pos)
{
	bool succ = true;
	succ = LoadBodyDefs(char_file, mBodyDefs);

	int num_joints = GetNumJoints();
	mBodyParts.resize(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		auto& curr_part = mBodyParts[j];
		double mFriction = 0.8;
		BuildBodyPart(j, root_pos, mFriction, curr_part);

		if (curr_part != nullptr)
		{
			curr_part->UpdateContact(cWorld::eContactFlagCharacter, cWorld::eContactFlagAll);
			curr_part->DisableDeactivation();
		}
	}

	return true;
}

tVector cKinSimCharacter::CalcCOM() const
{
	tVector com = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_com = part->GetPos();

			com += mass * curr_com;
			total_mass += mass;
		}
	}
	com /= total_mass;
	return com;
}

tVector cKinSimCharacter::GetRootVel() const
{
	const auto& part = mBodyParts[0];
	tVector curr_root_vel = part->GetLinearVelocity();
	return curr_root_vel;
}

void cKinSimCharacter::BuildBodyPart(int part_id, const tVector& root_pos, double friction, std::shared_ptr<cSimObj>& out_part)
{
	cKinTree::eBodyShape shape = cKinTree::GetBodyShape(mBodyDefs, part_id);
	switch (shape)
	{
		case cKinTree::eBodyShapeBox:
			BuildBoxPart(part_id, root_pos, friction, out_part);
			break;
		case cKinTree::eBodyShapeCapsule:
			BuildCapsulePart(part_id, root_pos, friction, out_part);
			break;
		case cKinTree::eBodyShapeSphere:
			BuildSpherePart(part_id, root_pos, friction, out_part);
			break;
		case cKinTree::eShapeCylinder:
			BuildCylinderPart(part_id, root_pos, friction, out_part);
			break;
		default:
			std::cout << "Body part shape not supported" << std::endl;
			out_part = nullptr;
			break;
	}
}

void cKinSimCharacter::BuildBoxPart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimBox::tParams params;
	params.mSize = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimBox> box = std::unique_ptr<cSimBox>(new cSimBox());

	short col_group = 5;
	short col_mask = 1;
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cKinSimCharacter::BuildCapsulePart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimCapsule::tParams params;
	tVector size_params = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mHeight = size_params(1);
	params.mRadius = size_params(0);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimCapsule> box = std::unique_ptr<cSimCapsule>(new cSimCapsule());

	short col_group = 5;
	short col_mask = 1;
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cKinSimCharacter::BuildSpherePart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimSphere::tParams params;
	tVector size_params = cKinTree::GetBodySize(mBodyDefs, part_id);
	// params.mHeight = size_params(1);
	params.mRadius = size_params(0);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimSphere> box = std::unique_ptr<cSimSphere>(new cSimSphere());

	short col_group = 5;
	short col_mask = 1;
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cKinSimCharacter::BuildCylinderPart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimCylinder::tParams params;
	tVector size_params = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mHeight = size_params(1);
	params.mRadius = size_params(0);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimCylinder> box = std::unique_ptr<cSimCylinder>(new cSimCylinder());

	short col_group = 5;
	short col_mask = 1;
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

bool cKinSimCharacter::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const
{
	bool succ = cKinTree::LoadBodyDefs(char_file, out_body_defs);
	int num_joints = GetNumJoints();
	int num_body_defs = static_cast<int>(out_body_defs.rows());
	assert(num_joints == num_body_defs);
	return succ;
}

bool cKinSimCharacter::HasDrawShapes() const
{
	return mDrawShapeDefs.size() > 0;
}
const Eigen::MatrixXd& cKinSimCharacter::GetBodyDefs() const
{
	return mBodyDefs;
}

const Eigen::MatrixXd& cKinSimCharacter::GetDrawShapeDefs() const
{
	return mDrawShapeDefs;
}
size_t cKinSimCharacter::GetNumBodyParts() const
{
	return mBodyParts.size();
}
bool cKinSimCharacter::IsValidBodyPart(int idx) const
{
	return mBodyParts[idx] != nullptr;
}
const std::shared_ptr<cSimObj>& cKinSimCharacter::GetBodyPart(int idx) const
{
	return mBodyParts[idx];
}
void cKinSimCharacter::SetGround(std::shared_ptr<cGround> ground)
{
	mGround = ground;
}
tVector cKinSimCharacter::GetBodyPartVel(int idx) const
{
	auto& part = GetBodyPart(idx);
	tVector vel = part->GetLinearVelocity();
	return vel;
}
void cKinSimCharacter::SetBodyPartVel(int idx, tVector vel) const
{
	auto& part = GetBodyPart(idx);
	part->SetLinearVelocity(vel);
}
double cKinSimCharacter::SampleGroundHeight(const tVector& pos) const
{
	double h = 0;
	if (mGround != nullptr)
	{
		h = mGround->SampleHeight(pos);
	}
	return h;
}
bool cKinSimCharacter::SampleGroundHeightVel(const tVector& pos, double& out_h, tVector& out_vel) const
{
	bool valid_sample = false;
	out_h = 0;
	out_vel.setZero();

	if (mGround != nullptr)
	{
		mGround->SampleHeightVel(pos, out_h, out_vel, valid_sample);
	}
	return valid_sample;
}

void cKinSimCharacter::BuildKinCharPoliStatePose(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = GetJointMat();
	const Eigen::VectorXd& pose = GetPose();
	tMatrix origin_trans = BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	tVector root_pos = GetRootPos();
	double ground_h = SampleGroundHeight(root_pos);
	tVector root_pos_rel = root_pos;
	root_pos_rel[1] -= ground_h;

	root_pos_rel[3] = 1;
	root_pos_rel = origin_trans * root_pos_rel;
	root_pos_rel[3] = 0;

	out_pose = Eigen::VectorXd::Zero(GetPoseFeatureSize());
	out_pose[0] = root_pos_rel[1];
	int num_parts = GetNumBodyParts();

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	int idx = 1;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (IsValidBodyPart(part_id))
		{
			const auto& curr_part = GetBodyPart(part_id);
			tVector curr_pos = curr_part->GetPos();
			curr_pos[1] -= ground_h;

			curr_pos[3] = 1;
			curr_pos = origin_trans * curr_pos;
			curr_pos[3] = 0;
			curr_pos -= root_pos_rel;

			out_pose.segment(idx, pos_dim) = curr_pos.segment(0, pos_dim);
			idx += pos_dim;

#if defined(ENABLE_MAX_STATE)
			if (Is3D())
			{
				tQuaternion curr_quat = curr_part->GetRotation();
				curr_quat = origin_quat * curr_quat;

				if (flip_stance)
				{
					curr_quat = cMathUtil::MirrorQuaternion(curr_quat, cMathUtil::eAxisZ);
				}

				if (curr_quat.w() < 0)
				{
					curr_quat.w() *= -1;
					curr_quat.x() *= -1;
					curr_quat.y() *= -1;
					curr_quat.z() *= -1;
				}
				out_pose.segment(idx, rot_dim) = cMathUtil::QuatToVec(curr_quat).segment(0, rot_dim);
				idx += rot_dim;
			}
#endif
		}
	}
}

double cKinSimCharacter::GetPhase() const
{
	double phase = (mTime) / mMotion.GetDuration();
	phase -= static_cast<int>(phase);
	phase = (phase < 0) ? (1 + phase) : phase;
	return phase;
}

void cKinSimCharacter::BuildKinCharPoliStateVel(Eigen::VectorXd& out_vel) const
{
	out_vel.resize(GetVelFeatureSize());
	int num_parts = GetNumBodyParts();

	const Eigen::MatrixXd& joint_mat = GetJointMat();
	const Eigen::VectorXd& pose = GetPose();
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, pose);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	tVector root_pos = GetRootPos();
	double ground_h = 0;
	tVector ground_vel = tVector::Zero();
	bool valid_sample = SampleGroundHeightVel(root_pos, ground_h, ground_vel);
	assert(valid_sample);

	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);

		const auto& curr_part = GetBodyPart(part_id);
		tVector curr_vel = curr_part->GetLinearVelocity();
		curr_vel -= ground_vel;
		curr_vel = origin_trans * curr_vel;

		out_vel.segment(idx, pos_dim) = curr_vel.segment(0, pos_dim);
		idx += pos_dim;

#if defined(ENABLE_MAX_STATE)
		if (Is3D())
		{
			tVector curr_ang_vel = curr_part->GetAngularVelocity();
			curr_ang_vel = origin_trans * curr_ang_vel;
			if (flip_stance)
			{
				curr_ang_vel = -curr_ang_vel;
			}

			out_vel.segment(idx, rot_dim - 1) = curr_ang_vel.segment(0, rot_dim - 1);
			idx += rot_dim - 1;
		}
#endif
	}
}

int cKinSimCharacter::GetPosFeatureDim() const
{
	int pos_dim = cKinTree::gPosDim - 1;
	if (Is3D())
	{
		pos_dim = cKinTree::gPosDim;
	}
	return pos_dim;
}
int cKinSimCharacter::GetRotFeatureDim() const
{
	int rot_dim = 0;
	if (Is3D())
	{
#if defined(ENABLE_MAX_STATE)
		rot_dim = cKinTree::gRotDim;
#endif
	}
	return rot_dim;
}
bool cKinSimCharacter::Is3D() const
{
	const auto& world = mWorld;
	cWorld::eSimMode sim_mode = world->GetSimMode();
	return sim_mode == cWorld::eSimMode3D;
}

size_t cKinSimCharacter::GetPoseFeatureSize() const
{
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();
	return GetNumBodyParts() * (pos_dim + rot_dim) + 1; // +1 for root y
}

size_t cKinSimCharacter::GetVelFeatureSize() const
{
	int size = 0;
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	if (Is3D())
	{
		size = GetNumBodyParts() * (pos_dim + rot_dim - 1);
	}
	else
	{
		size = GetNumBodyParts() * pos_dim;
	}
	return size;
}

int cKinSimCharacter::RetargetJointID(int joint_id) const
{
	return joint_id;
}
bool cKinSimCharacter::FlipStance() const
{
	return false;
}

void cKinSimCharacter::BuildKinCharPoliState(Eigen::VectorXd& out_state) const
{
	int state_size = GetPoseFeatureSize() + GetVelFeatureSize();
	out_state.resize(state_size);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	BuildKinCharPoliStatePose(pose);
	BuildKinCharPoliStateVel(vel);

	// TODO: In debug mode ground has 200 samples in it that EIGEN will complain about assigning to a vector of size 0.
	out_state.segment(0, GetPoseFeatureSize()) = pose;
	out_state.segment(GetPoseFeatureSize(), GetVelFeatureSize()) = vel;
}

void cKinSimCharacter::SetPose(const Eigen::VectorXd& pose)
{
	cKinCharacter::SetPose(pose);

	for (size_t i = 0; i < mBodyParts.size(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			auto& curr_part = mBodyParts[i];
			tVector axis;
			double theta;
			cKinTree::CalcBodyPartRotation(mJointMat, mBodyDefs, mPose, i, axis, theta);
			tVector pos = cKinTree::CalcBodyPartPos(mJointMat, mBodyDefs, mPose, i);

			curr_part->SetPos(pos);
			curr_part->SetRotation(axis, theta);
		}
	}

}

void cKinSimCharacter::SetVel(const Eigen::VectorXd& vel)
{
	cKinCharacter::SetVel(vel);

	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b))
		{
			auto& part = GetBodyPart(b);
			int joint_id = b;

			cSpAlg::tSpVec sv = cRBDUtil::CalcWorldVel(mJointMat, mPose, vel, joint_id);
			tVector pos = GetBodyPart(b)->GetPos();
			cSpAlg::tSpTrans world_to_pt = cSpAlg::BuildTrans(pos);
			sv = cSpAlg::ApplyTransM(world_to_pt, sv);

			tVector com_omega = cSpAlg::GetOmega(sv);
			tVector com_vel = cSpAlg::GetV(sv);
			part->SetAngularVelocity(com_omega);
			part->SetLinearVelocity(com_vel);
		}
	}

}


void cKinSimCharacter::setTimeWarping(double warp)
{
	timeWarping = warp;
}

double cKinSimCharacter::getTimeWarping()
{
	return timeWarping;
}
