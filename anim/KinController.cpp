#include "anim/KinController.h"
#include "anim/KinCharacter.h"
#include "util/FileUtil.h"

cKinController::cKinController()
{
	mTime = 0;
}

cKinController::~cKinController()
{
}

void cKinController::Init(cKinCharacter* character)
{
	assert(character != nullptr);
	Clear();
	mChar = character;
}

void cKinController::Reset()
{
	mTime = 0;
}

void cKinController::Clear()
{
	mChar = nullptr;
	mTime = 0;
}

void cKinController::Update(double time_step)
{
	mTime += time_step;
}

void cKinController::CalcPose(double time, Eigen::VectorXd& out_pose) const
{
}

void cKinController::SetTime(double time)
{
	mTime = time;
}

double cKinController::GetTime() const
{
	return mTime;
}

bool cKinController::IsMotionOver() const
{
	return false;
}

size_t cKinController::getMotionID() const
{
	return 0;
}

void cKinController::setMotionID(size_t id)
{
	/// Does nothings....
}

int cKinController::GetNumMotions() const
{
	return 1;
}
