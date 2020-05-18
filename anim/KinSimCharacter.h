#pragma once

#include "anim/KinCharacter.h"
#include "anim/Motion.h"
#include "anim/KinController.h"
#include "sim/World.h"
#include "sim/Ground.h"
#include "sim/SimObj.h"
#include "sim/Joint.h"
#include "sim/CharController.h"

#define ENABLE_MAX_STATE

class cKinSimCharacter : public cKinCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cKinSimCharacter();
	virtual ~cKinSimCharacter();

	virtual bool Init(const std::string& char_file, const std::string& motion_file);
	virtual bool Init(const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void SetVel(const Eigen::VectorXd& vel);
	virtual tVector GetRootVel() const;

	virtual void BuildKinCharPoliStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildKinCharPoliStateVel(Eigen::VectorXd& out_vel) const;
	virtual void BuildKinCharPoliState(Eigen::VectorXd& out_state) const;
	virtual size_t GetPoseFeatureSize() const;
	virtual size_t GetVelFeatureSize() const;

	/// For creating sim parts of character
	virtual bool BuildBody(std::shared_ptr<cWorld> world);
	virtual bool BuildSimBody(const std::string& char_file, const tVector& root_pos);
	virtual void BuildBodyPart(int part_id, const tVector& root_pos, double friction, std::shared_ptr<cSimObj>& out_part);
	virtual void BuildBoxPart(int part_id, const tVector& root_pos, double friction,
									std::shared_ptr<cSimObj>& out_part);
	virtual void BuildCapsulePart(int part_id, const tVector& root_pos, double friction,
									std::shared_ptr<cSimObj>& out_part);
	virtual void BuildSpherePart(int part_id, const tVector& root_pos, double friction,
										std::shared_ptr<cSimObj>& out_part);
	virtual void BuildCylinderPart(int part_id, const tVector& root_pos, double friction,
										std::shared_ptr<cSimObj>& out_part);
	virtual bool LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const;


	virtual const Eigen::MatrixXd& GetDrawShapeDefs() const;
	virtual const Eigen::MatrixXd& GetBodyDefs() const;
	virtual bool HasDrawShapes() const;
	virtual size_t GetNumBodyParts() const;
	virtual bool IsValidBodyPart(int idx) const;
	virtual const std::shared_ptr<cSimObj>& GetBodyPart(int idx) const;
	virtual tVector GetBodyPartVel(int idx) const;
	virtual void SetBodyPartVel(int idx, tVector vel) const;
	virtual double SampleGroundHeight(const tVector& pos) const;
	virtual bool SampleGroundHeightVel(const tVector& pos, double& out_h, tVector& out_vel) const;
	virtual void SetGround(std::shared_ptr<cGround> ground);
	virtual void updateBodyLinkVelocities();
	virtual int GetPosFeatureDim() const;
	virtual int GetRotFeatureDim() const;
	virtual bool Is3D() const;
	virtual int RetargetJointID(int joint_id) const;
	virtual bool FlipStance() const;
	virtual tVector CalcCOM() const;
	virtual double GetPhase() const;

	virtual void setTimeWarping(double warp);
	virtual double getTimeWarping();

	virtual void CalcVel(double time, Eigen::VectorXd& out_vel) const;

protected:
	std::shared_ptr<cWorld> mWorld;
	std::vector<std::shared_ptr<cSimObj>> mBodyParts;
	std::vector<cJoint, Eigen::aligned_allocator<cJoint>> mJoints;
	Eigen::MatrixXd mDrawShapeDefs;
	Eigen::MatrixXd mBodyDefs;
	std::string charFile;
	std::shared_ptr<cGround> mGround;

	double timeWarping;

};
